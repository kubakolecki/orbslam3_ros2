#include "stereo-slam-node.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


#include <geometry_msgs/msg/point32.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>


#include <iostream>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <ranges>

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
    georeferencedStereoPublisher = this->create_publisher<ros_common_messages::msg::GeoreferencedStereoImage>("orbslam3/georeferenced_stereo_image", 10);
    pathPublisher = this->create_publisher<nav_msgs::msg::Path>("orbslam3/path", 10);

    pathMsg.header.frame_id = "world";
    pathMsg.poses.reserve(4096);

    std::cout <<"subscribers created" <<std::endl;
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(12), left_sub, right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
    std::cout <<"callback registerd" <<std::endl;

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



    //RCLCPP_INFO(this->get_logger(), "Tracked keypoints: %zu , tracked landmarks: %zu", trackedKeypoints.size(), trackedLandmarks.size());



    pose = pose.inverse();

    const auto unit_quaternion = pose.unit_quaternion();
    const auto translation = pose.translation();

    //std::cout << "quaternion:\n";
    //std::cout << unit_quaternion <<"\n";
    //std::cout << "translation:\n";
    //std::cout << translation <<"\n";

    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header = msgLeft->header;

    //setting timestamp to zero (debugging purposes)
    //poseMsg.header.stamp.sec = 0;
    //poseMsg.header.stamp.nanosec = 0;

    poseMsg.header.frame_id = "world";
    poseMsg.pose.position.x = translation(0);
    poseMsg.pose.position.y = translation(1);
    poseMsg.pose.position.z = translation(2);
    poseMsg.pose.orientation.x = unit_quaternion.x();
    poseMsg.pose.orientation.y = unit_quaternion.y();
    poseMsg.pose.orientation.z = unit_quaternion.z();
    poseMsg.pose.orientation.w = unit_quaternion.w();
    posePublisher->publish(poseMsg);

    pathMsg.poses.push_back(poseMsg);
    pathPublisher->publish(pathMsg);



    //hasFirstPoseBeenPublished = true;
    

    //auto end = std::chrono::steady_clock::now();
    //auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //std::cout << "Elapsed time: " << duration_ms.count() << " ms\n";


    Eigen::Vector3f differenceInPosition = translation - lastPublishedPosition;
    float distanceMoved = differenceInPosition.norm();

    if (distanceMoved > distanceThresholdToPublishStereoImage)
    {
        ros_common_messages::msg::GeoreferencedStereoImage geoStereoMsg;
        //geoStereoMsg.header = msgLeft->header;
        //geoStereoMsg.header.frame_id = "world";
        geoStereoMsg.pose = poseMsg;

        //old implementation: feding the original images, not the ortorectified ones
        //geoStereoMsg.image_left = *msgLeft;
        //geoStereoMsg.image_right = *msgRight;


        //new implementation: we need to fed ortorectfied images as the keypoints are tracked on the ortorectified images,
        //so we need to fed the same images to the georeferenced stereo image message, otherwise the keypoints and depth information will not be correct
        const auto imageFedToTrackerLeft = m_SLAM->GetImageFedToTrackerLeft(); //TODO can call those lines direct inside the call of cv_bridge::CvImage
        const auto imageFedToTrackerRight = m_SLAM->GetImageFedToTrackerRight();
        geoStereoMsg.image_left = *cv_bridge::CvImage(msgLeft->header, "mono8", imageFedToTrackerLeft).toImageMsg();
        geoStereoMsg.image_right = *cv_bridge::CvImage(msgRight->header, "mono8", imageFedToTrackerRight).toImageMsg();

        //auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        
        lastPublishedPosition = translation;

        const auto trackedLandmarks = m_SLAM->GetTrackedMapPoints();
        const auto trackedKeypoints = m_SLAM->GetTrackedKeyPointsUn();

        tf2::Transform transformationFromCameraToWorld;
        tf2::fromMsg(geoStereoMsg.pose.pose, transformationFromCameraToWorld);
        const tf2::Transform transformationFromWorldToCamera = transformationFromCameraToWorld.inverse();

        geoStereoMsg.sparse_depth_information.points.reserve(trackedLandmarks.size());


        //RCLCPP_INFO(this->get_logger(), "Preparing georeferenced stereo image with %zu tracked landmarks.", trackedLandmarks.size());
        //RCLCPP_INFO(this->get_logger(), "Preparing georeferenced stereo image with %zu tracked keypoints.", trackedKeypoints.size());
        /*
        std::ranges::transform(trackedLandmarks, trackedKeypoints, std::back_inserter(geoStereoMsg.sparse_depth_information.points),
            [transformationFromWorldToCamera=transformationFromWorldToCamera](ORB_SLAM3::MapPoint* mapPoint, const cv::KeyPoint& keypoint)
            {   
                if (mapPoint)
                {
                    RCLCPP_INFO(rclcpp::get_logger("StereoSlamNode"), "Processing MapPoint id");
                    const Eigen::Vector3d pointInWorld = mapPoint->GetWorldPos().cast<double>();
                    RCLCPP_INFO(rclcpp::get_logger("StereoSlamNode"), "MapPoint position in world: x=%f, y=%f, z=%f", pointInWorld(0), pointInWorld(1), pointInWorld(2));
                    tf2::Vector3 pointInCameraTf = transformationFromWorldToCamera * tf2::Vector3{pointInWorld(0), pointInWorld(1), pointInWorld(2)};
                    geometry_msgs::msg::Point32 point;
                    point.x = keypoint.pt.x;
                    point.y = keypoint.pt.y;
                    point.z = pointInCameraTf.z();    
                    return point;
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("StereoSlamNode"), "Null MapPoint encountered while preparing georeferenced stereo image.");
                    
                    if (mapPoint == nullptr)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("StereoSlamNode"), "MapPoint pointer is null.");
                    }

                    geometry_msgs::msg::Point32 point;
                    point.x = keypoint.pt.x;
                    point.y = keypoint.pt.y;
                    point.z = -1.0f; // Indicate invalid depth
                    return point;
                }

            });
        */

        auto valid_map_point_indices = std::views::iota(size_t{0}, trackedLandmarks.size()) | std::views::filter([&trackedLandmarks](size_t i){ return trackedLandmarks[i] != nullptr; });
        std::ranges::transform(valid_map_point_indices, std::back_inserter(geoStereoMsg.sparse_depth_information.points),
            [&trackedLandmarks, &trackedKeypoints, transformationFromWorldToCamera=transformationFromWorldToCamera](size_t i)
            {
                ORB_SLAM3::MapPoint* mapPoint = trackedLandmarks[i];
                const cv::KeyPoint& keypoint = trackedKeypoints[i];

                const Eigen::Vector3d pointInWorld = mapPoint->GetWorldPos().cast<double>();
                //TODO: we need to compute only z in camera frame, so we can optimize this by only computing the z component instead of the full transformation
                tf2::Vector3 pointInCameraTf = transformationFromWorldToCamera * tf2::Vector3{pointInWorld(0), pointInWorld(1), pointInWorld(2)};
                geometry_msgs::msg::Point32 point;
                point.x = keypoint.pt.x;
                point.y = keypoint.pt.y;
                point.z = pointInCameraTf.z();
                return point;
            });

        geoStereoMsg.sparse_depth_information.points.shrink_to_fit();
        
        // Extracting depths for debugging purposes: TODO: remove this when not needed anymore
        std::vector<double> depths;
        depths.reserve(geoStereoMsg.sparse_depth_information.points.size());
        for (const auto& point : geoStereoMsg.sparse_depth_information.points)
        {            
            depths.push_back(point.z);
        }

        RCLCPP_INFO(this->get_logger(), "Number of landmarks is = %zu, number of keypoints is = %zu, number of depth points in georeferenced stereo image is = %zu", trackedLandmarks.size(), trackedKeypoints.size(), geoStereoMsg.sparse_depth_information.points.size());

        RCLCPP_INFO(this->get_logger(), "Publishing georeferenced stereo image with %zu map points.", geoStereoMsg.sparse_depth_information.points.size());
        georeferencedStereoPublisher->publish(geoStereoMsg);
        
        //writeMapPointsToFile(trackedLandmarks, trackedKeypoints, previousLeftImage);
        //writeMapPointsToFile(trackedLandmarks, trackedKeypoints, imageFedToTrackerLeft, depths);

    }



}

void StereoSlamNode::writeMapPointsToFile(const std::vector<ORB_SLAM3::MapPoint*>& mapPoints, const std::vector<cv::KeyPoint>& keyPoints,const cv::Mat& image, const vector<double>& depths)
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    const std::string datetime = oss.str();

    const std::string filenameMapPoints = "MapPoints_" + datetime + ".txt";
    const std::string filenameKeyPoints = "KeyPoints_" + datetime + ".txt";

    const std::string headerMapPoints{"id,observations,is_bad,x,y,z,kp_x,kp_y\n"};
    const std::string headerKeyPoints{"x,y\n"};

    std::ofstream fileMapPoints;
    fileMapPoints.open(filenameMapPoints);



    fileMapPoints <<std::boolalpha;
    fileMapPoints <<std::fixed << std::setprecision(4);

    fileMapPoints << headerMapPoints;


    std::vector<cv::KeyPoint> keyPointsForMapPoints;
    //std::vector<double> depthsForMapPoints;
    keyPointsForMapPoints.reserve(mapPoints.size());
    int index{0};
    for (const auto mapPoint: mapPoints)
    {
        if (mapPoint)
        {
            Eigen::Vector3f positionInWorld{mapPoint->GetWorldPos()};
            fileMapPoints << mapPoint->isBad() <<",";
            fileMapPoints << mapPoint->Observations() <<",";
            fileMapPoints << positionInWorld(0) <<","<<positionInWorld(1)<<","<<positionInWorld(2);
            fileMapPoints <<","<< keyPoints[index].pt.x <<","<< keyPoints[index].pt.y;
            fileMapPoints << "\n";

            keyPointsForMapPoints.push_back(keyPoints[index]);
            //depthsForMapPoints.push_back(depths[index]);
        }
        index++;
    }

    fileMapPoints.close();
    keyPointsForMapPoints.shrink_to_fit();

    //draw keypoints in image and save image
    cv::Mat imageWithKeypoints;
    cv::drawKeypoints(image, keyPointsForMapPoints, imageWithKeypoints, cv::Scalar(0,255,0));

    //printing depths on keypoints
    /*
    for (size_t i=0; i<keyPointsForMapPoints.size(); ++i)
    {
        //const double depth = depthsForMapPoints[i];
        const double depth = depths[i];
        std::ostringstream depthText;
        depthText << std::fixed << std::setprecision(1) << depth;
        cv::putText(imageWithKeypoints, depthText.str(), keyPointsForMapPoints[i].pt + cv::Point2f(1.0,1.0), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,0,255), 2, cv::LINE_AA);
    }
    */

    const std::string imageFilename = "MapPointsImage_" + datetime + ".png";
    cv::imwrite(imageFilename, imageWithKeypoints);


    

    /*
    std::ofstream fileKeyPoints;
    fileKeyPoints.open(filenameKeyPoints);
    fileKeyPoints << headerKeyPoints;
    fileKeyPoints <<std::fixed <<std::setprecision(2);
    for (const auto keyPoint: keyPoints)
    {
        fileKeyPoints << keyPoint.pt.x <<","<<keyPoint.pt.y <<"\n";
    }

    fileKeyPoints.close();
    */




}



