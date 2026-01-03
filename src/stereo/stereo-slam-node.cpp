#include "stereo-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;
    std::cout<<"do rectify: " << boolalpha << doRectify <<std::endl;



    if (doRectify){
        std::cout <<"do rectify is true!" <<std::endl;
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    std::cout <<"creating subscribers..." <<std::endl;
    //left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/left");
    //std::cout <<"left image subscriber created" <<std::endl;
    //right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/right");
    //std::cout <<"right image subscriber created" <<std::endl;
    //left_sub.subscribe(this,"camera/left", rmw_qos_profile_sensor_data);
    //right_sub.subscribe(this,"camera/right", rmw_qos_profile_sensor_data);

    auto paramOutputImageScalingDescription = rcl_interfaces::msg::ParameterDescriptor{};
    paramOutputImageScalingDescription.description = "Distance threshold in meters to publish the next stereo image with georeference.";
    this->declare_parameter<float>("distance_threshold_to_publish_stereo_image", 0.25, paramOutputImageScalingDescription);
    distanceThresholdToPublishStereoImage = this->get_parameter("distance_threshold_to_publish_stereo_image").as_double();


    left_sub.subscribe(this,"camera/left");
    right_sub.subscribe(this,"camera/right");

    posePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("orbslam3/pose", 10);
    georeferencedStereoPublisher = this->create_publisher<orbslam3::msg::GeoreferencedStereoImage>("orbslam3/georeferenced_stereo_image", 10);

    std::cout <<"subscribers created" <<std::endl;
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(12), left_sub, right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
    std::cout <<"callback registerd" <<std::endl;


    positionsOfPublishedStereoImages.reserve(4096);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string datetime = oss.str();

    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_" + datetime + ".txt" );
    m_SLAM->SaveTrajectoryEuRoC("FullTrajectory_" + datetime + ".txt");

    positionsOfPublishedStereoImages.shrink_to_fit();


    writePositionsOfPublishedStereoImagesToFile("PublishedStereoImagePositions_" + datetime + ".txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    //auto start = std::chrono::steady_clock::now();
    
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f pose;

    if (doRectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        pose = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    }
    else
    {
        pose = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    }

    pose = pose.inverse();

    const auto unit_quaternion = pose.unit_quaternion();
    const auto translation = pose.translation();

    //std::cout << "quaternion:\n";
    //std::cout << unit_quaternion <<"\n";
    //std::cout << "translation:\n";
    //std::cout << translation <<"\n";

    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header = msgLeft->header;
    poseMsg.header.frame_id = "world";
    poseMsg.pose.position.x = translation(0);
    poseMsg.pose.position.y = translation(1);
    poseMsg.pose.position.z = translation(2);
    poseMsg.pose.orientation.x = unit_quaternion.x();
    poseMsg.pose.orientation.y = unit_quaternion.y();
    poseMsg.pose.orientation.z = unit_quaternion.z();
    poseMsg.pose.orientation.w = unit_quaternion.w();
    posePublisher->publish(poseMsg);
    //hasFirstPoseBeenPublished = true;
    

    //auto end = std::chrono::steady_clock::now();
    //auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //std::cout << "Elapsed time: " << duration_ms.count() << " ms\n";


    Eigen::Vector3f differenceInPosition = translation - lastPublishedPosition;
    float distanceMoved = differenceInPosition.norm();

    if (distanceMoved > distanceThresholdToPublishStereoImage)
    {
        orbslam3::msg::GeoreferencedStereoImage geoStereoMsg;
        //geoStereoMsg.header = msgLeft->header;
        //geoStereoMsg.header.frame_id = "world";
        geoStereoMsg.pose = poseMsg;
        geoStereoMsg.image_left = *msgLeft;
        geoStereoMsg.image_right = *msgRight;
        georeferencedStereoPublisher->publish(geoStereoMsg);
        lastPublishedPosition = translation;

        positionsOfPublishedStereoImages.push_back(translation);
    }

}

void StereoSlamNode::writePositionsOfPublishedStereoImagesToFile(const string &filename)
{
    std::ofstream file;
    file.open(filename);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open file %s for writing positions of published stereo images.", filename.c_str());
        return;
    }

    file << std::fixed << std::setprecision(6);
    for (const auto& position : positionsOfPublishedStereoImages)
    {
        file << position(0) << " " << position(1) << " " << position(2) << "\n";
    }

    file.close();


}
