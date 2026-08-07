#include "stereo-slam-node.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>


#include <iostream>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <ranges>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    auto paramDoPublishStereorectifiedImagesDescription{rcl_interfaces::msg::ParameterDescriptor{}};
    auto paramDoWritePosesToTextFileDescription{rcl_interfaces::msg::ParameterDescriptor{}};
    auto paramPathToSavePosesDescription{rcl_interfaces::msg::ParameterDescriptor{}};
    auto paramDistanceThresholdToPublishStereoImageDescription{rcl_interfaces::msg::ParameterDescriptor{}};
    auto paramDoPublishTrackedKeypointsVisualizationDescription{rcl_interfaces::msg::ParameterDescriptor{}};
    auto paramDoSaveLocalMapToFileDescription{rcl_interfaces::msg::ParameterDescriptor{}};
    auto paramPathToSaveLocalMapDescription{rcl_interfaces::msg::ParameterDescriptor{}};

    
    paramDoPublishStereorectifiedImagesDescription.description = "If True, stereorectified images provided by orbslam are published. The timestamp is same as timestamp of computed pose.";
    paramDoWritePosesToTextFileDescription.description = "If True, poses are written to a text file.";
    paramPathToSavePosesDescription.description = "The path to file where the poses will be saved if do_write_poses_to_text_file is true.";
    paramDistanceThresholdToPublishStereoImageDescription.description = "Distance threshold in meters to publish the next stereo image with georeference.";
    paramDoPublishTrackedKeypointsVisualizationDescription.description = "If true, the visualization of keypoints is publshed.";
    paramDoSaveLocalMapToFileDescription.description = "If true, the local map is saved to a file.";
    paramPathToSaveLocalMapDescription.description = "The path where the local map will be saved if do_save_local_map_to_file is true.";

    this->declare_parameter<bool>("do_publish_stereorectified_images", "true", paramDoPublishStereorectifiedImagesDescription);
    this->declare_parameter<bool>("do_write_poses_to_text_file", "true", paramDoWritePosesToTextFileDescription);
    this->declare_parameter<std::string>("path_to_save_poses", "poses.txt", paramPathToSavePosesDescription);
    this->declare_parameter<float>("distance_threshold_to_publish_stereo_image", 0.25, paramDistanceThresholdToPublishStereoImageDescription);
    this->declare_parameter<bool>("do_publish_tracked_keypoints_visualization", "true", paramDoPublishTrackedKeypointsVisualizationDescription);
    this->declare_parameter<bool>("do_save_local_map_to_file", "true", paramDoSaveLocalMapToFileDescription);
    this->declare_parameter<std::string>("path_to_save_local_map", "", paramPathToSaveLocalMapDescription);

    doPublishStereorectifiedImages = this->get_parameter("do_publish_stereorectified_images").as_bool();
    doWritePosesToTextFile = this->get_parameter("do_write_poses_to_text_file").as_bool();
    distanceThresholdToPublishStereoImage = this->get_parameter("distance_threshold_to_publish_stereo_image").as_double();
    doPublishTrackedKeypointsVisualization = this->get_parameter("do_publish_tracked_keypoints_visualization").as_bool();
    doSaveLocalMapToFile = this->get_parameter("do_save_local_map_to_file").as_bool();
    pathToSaveLocalMap = this->get_parameter("path_to_save_local_map").as_string();
    pathToSavePoses = this->get_parameter("path_to_save_poses").as_string();
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

    //std::cout <<"creating subscribers..." <<std::endl;
    //left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/left");
    //std::cout <<"left image subscriber created" <<std::endl;
    //right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/right");
    //std::cout <<"right image subscriber created" <<std::endl;
    //left_sub.subscribe(this,"camera/left", rmw_qos_profile_sensor_data);
    //right_sub.subscribe(this,"camera/right", rmw_qos_profile_sensor_data);


    left_sub.subscribe(this,"camera/left");
    right_sub.subscribe(this,"camera/right");

    posePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("orbslam3/pose", 10);
    georeferencedStereoPublisher = this->create_publisher<ros_common_messages::msg::GeoreferencedStereoImage>("orbslam3/georeferenced_stereo_image", 10);
    pathPublisher = this->create_publisher<nav_msgs::msg::Path>("orbslam3/path", 10);
    stereoRectifiedLeftPublisher = this->create_publisher<sensor_msgs::msg::Image>("orbslam3/stereo_rectified_left", 10);
    stereoRectifiedRightPublisher = this->create_publisher<sensor_msgs::msg::Image>("orbslam3/stereo_rectified_right", 10);
    keypointVisualizationPublisher = this->create_publisher<sensor_msgs::msg::Image>("orbslam3/keypoint_visualization", 10);

    pathMsg.header.frame_id = "world";
    pathMsg.poses.reserve(4096);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(12), left_sub, right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    if (doWritePosesToTextFile)
    {
        //std::cout <<"doWritePosesToTextFile is true, opening file for writing poses..." <<std::endl;
        //std::time_t t = std::time(nullptr);
        //std::tm tm = *std::localtime(&t);
        //std::ostringstream oss;
        //oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        //const std::string datetime = oss.str();
        //const std::string filename = "Poses_" + datetime + ".txt";
        posesOutputFile.open(pathToSavePoses);
        posesOutputFile << "timestamp_sec,timestamp_nanosec,position_x,position_y,position_z,quaternion_w,quaternion_x,quaternion_y,quaternion_z\n";
    }

    if (doSaveLocalMapToFile)
    {
        if (!std::filesystem::exists(pathToSaveLocalMap))
        {
            RCLCPP_INFO(this->get_logger(), "Directory %s does not exist. Creating directory.", pathToSaveLocalMap.c_str());   
            std::filesystem::create_directories(pathToSaveLocalMap);    
        }
        
    }

    base = m_SLAM->GetUpdatedBase();
    cameraConstantAfterStereorectification = m_SLAM->GetParamOfFirstCamera(0);

    cameraMatrixLeftAfterStereorectification.data.reserve(9);
    cameraMatrixRightAfterStereorectification.data.reserve(9);
    transformationRghtToLeftAfterStereorectification.data.reserve(16);
    for (auto i{0}; i<9;++i)
    {
        cameraMatrixLeftAfterStereorectification.data.push_back(0.0f);
        cameraMatrixRightAfterStereorectification.data.push_back(0.0f);
    }

    for (auto i{0}; i<16;++i)
    {
        transformationRghtToLeftAfterStereorectification.data.push_back(0.0f);
    }

    transformationRghtToLeftAfterStereorectification.data[0] = 1.0f;
    transformationRghtToLeftAfterStereorectification.data[5] = 1.0f;
    transformationRghtToLeftAfterStereorectification.data[10] = 1.0f;
    transformationRghtToLeftAfterStereorectification.data[15] = 1.0f;
    transformationRghtToLeftAfterStereorectification.data[3] = base;

    cameraMatrixLeftAfterStereorectification.data[0] = m_SLAM->GetParamOfFirstCamera(0);
    cameraMatrixLeftAfterStereorectification.data[4] = m_SLAM->GetParamOfFirstCamera(1);
    cameraMatrixLeftAfterStereorectification.data[2] = m_SLAM->GetParamOfFirstCamera(2);
    cameraMatrixLeftAfterStereorectification.data[5] = m_SLAM->GetParamOfFirstCamera(3);
    cameraMatrixLeftAfterStereorectification.data[8] = 1.0f;

    cameraMatrixRightAfterStereorectification.data[0] = m_SLAM->GetParamOfFirstCamera(0); //notice that after stereorectification in orbslam, both camera matrices should be the same
    cameraMatrixRightAfterStereorectification.data[4] = m_SLAM->GetParamOfFirstCamera(1);
    cameraMatrixRightAfterStereorectification.data[2] = m_SLAM->GetParamOfFirstCamera(2);
    cameraMatrixRightAfterStereorectification.data[5] = m_SLAM->GetParamOfFirstCamera(3);
    cameraMatrixRightAfterStereorectification.data[8] = 1.0f;

}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    if (doWritePosesToTextFile)
    {
        posesOutputFile.close();
    }

    // Save camera trajectory
    //std::time_t t = std::time(nullptr);
    //std::tm tm = *std::localtime(&t);
    //std::ostringstream oss;
    //oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    //std::string datetime = oss.str();

    //m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_" + datetime + ".txt" );

    const auto pathToSavePosesBaseName{pathToSavePoses.substr(0, pathToSavePoses.find_last_of('.'))};
    m_SLAM->SaveTrajectoryEuRoC(pathToSavePosesBaseName + "_final_orbsalm_trajectory.txt");

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

    //const auto timeSLAMEstimationStart {std::chrono::steady_clock::now()};

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

    //const auto timeSLAMEstimationEnd {std::chrono::steady_clock::now()};
    //const auto durationSLAMEstimation_ms {std::chrono::duration_cast<std::chrono::milliseconds>(timeSLAMEstimationEnd - timeSLAMEstimationStart)};
   


    //const auto timeOutputHandlingStart {std::chrono::steady_clock::now()};

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

    if (doWritePosesToTextFile)
    {
        posesOutputFile << poseMsg.header.stamp.sec << "," <<std::setfill('0') << std::setw(9) << poseMsg.header.stamp.nanosec << ",";
        posesOutputFile <<std::fixed << std::setprecision(6) << poseMsg.pose.position.x << "," << poseMsg.pose.position.y << "," << poseMsg.pose.position.z << ",";
        posesOutputFile <<std::fixed << std::setprecision(14) << poseMsg.pose.orientation.w << "," << poseMsg.pose.orientation.x << "," << poseMsg.pose.orientation.y << "," << poseMsg.pose.orientation.z << "\n";
    }


    pathMsg.poses.push_back(poseMsg);
    pathPublisher->publish(pathMsg);

    if (doPublishStereorectifiedImages)
    {
        stereoRectifiedLeftPublisher->publish(*cv_bridge::CvImage(poseMsg.header, "mono8", m_SLAM->GetImageFedToTrackerLeft()).toImageMsg());
        stereoRectifiedRightPublisher->publish(*cv_bridge::CvImage(poseMsg.header, "mono8", m_SLAM->GetImageFedToTrackerRight()).toImageMsg());
    }



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
        geoStereoMsg.sparse_depth_information.channels.emplace_back();
        geoStereoMsg.sparse_depth_information.channels.back().name = "uncertainty";
        geoStereoMsg.sparse_depth_information.channels.back().values.reserve(trackedLandmarks.size());

        auto valid_map_point_indices =  std::views::iota(size_t{0}, trackedLandmarks.size()) |
                                        std::views::filter([&trackedLandmarks](size_t i){ return trackedLandmarks[i] != nullptr; }) |
                                        std::views::filter([&trackedLandmarks](size_t i){ return trackedLandmarks[i]->nObs >= 4; }); //TODO: make this a parameter!

        for (const auto i : valid_map_point_indices)
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
            geoStereoMsg.sparse_depth_information.points.push_back(point);

            //belows formula for accuracy assums that accuracy of image point measurement is 1 px
            //this formula is Heuristic: it assumes that the more projectinos point has the lower the uncertainty should be
            float depthUncertainty{point.z * point.z / (cameraConstantAfterStereorectification*base)};
            depthUncertainty /= sqrt(static_cast<float> (mapPoint->nObs));
            geoStereoMsg.sparse_depth_information.channels.back().values.push_back(depthUncertainty);

            //std::cout << "projections: " << mapPoint->nObs << " depth: " << point.z << " uncertainty: " << depthUncertainty <<"\n";


        }

        /*
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
        */

        geoStereoMsg.sparse_depth_information.points.shrink_to_fit();
        geoStereoMsg.sparse_depth_information.channels.shrink_to_fit();

        geoStereoMsg.camera_matrix_left = cameraMatrixLeftAfterStereorectification;
        geoStereoMsg.camera_matrix_right = cameraMatrixRightAfterStereorectification;
        geoStereoMsg.right_to_left_transformation_matrix = transformationRghtToLeftAfterStereorectification;
        
        // Extracting depths for debugging purposes: TODO: remove this when not needed anymore
        //std::vector<double> depths;
        //depths.reserve(geoStereoMsg.sparse_depth_information.points.size());
        //for (const auto& point : geoStereoMsg.sparse_depth_information.points)
        //{            
        //    depths.push_back(point.z);
        //}

        //RCLCPP_INFO(this->get_logger(), "Number of landmarks is = %zu, number of keypoints is = %zu, number of depth points in georeferenced stereo image is = %zu", trackedLandmarks.size(), trackedKeypoints.size(), geoStereoMsg.sparse_depth_information.points.size());

        RCLCPP_INFO(this->get_logger(), "\033[36mPublishing georeferenced stereo image with %zu map points.\033[0m", geoStereoMsg.sparse_depth_information.points.size());
        georeferencedStereoPublisher->publish(geoStereoMsg);
        
        if (doSaveLocalMapToFile)
        {
            std::vector<tf2::Vector3> mapInCameraCoordinateSystem;
            mapInCameraCoordinateSystem.reserve(trackedLandmarks.size());
            std::ranges::transform(valid_map_point_indices, std::back_inserter(mapInCameraCoordinateSystem),
            [&trackedLandmarks, &transformationFromWorldToCamera](size_t i)
            {
                ORB_SLAM3::MapPoint* mapPoint = trackedLandmarks[i];
                const Eigen::Vector3d pointInWorld = mapPoint->GetWorldPos().cast<double>();
                tf2::Vector3 pointInCameraCs = transformationFromWorldToCamera * tf2::Vector3{pointInWorld(0), pointInWorld(1), pointInWorld(2)};
                return pointInCameraCs;
            });
            mapInCameraCoordinateSystem.shrink_to_fit();

            const auto pathFileLocalMap{std::filesystem::path{pathToSaveLocalMap} / (stampToString(geoStereoMsg.pose.header.stamp) + ".txt")};
            std::ofstream fileLocalMap{pathFileLocalMap};

            fileLocalMap << std::fixed << std::setprecision(5);
            for (const auto& point:  mapInCameraCoordinateSystem)
            {
                fileLocalMap << point.x() << "," << point.y() << "," << point.z() << "\n";
            }

            fileLocalMap.close();

        }

        
        //writeMapPointsToFile(trackedLandmarks, trackedKeypoints, previousLeftImage);
        //writeMapPointsToFile(trackedLandmarks, trackedKeypoints, imageFedToTrackerLeft, depths);

    }

    if (doPublishTrackedKeypointsVisualization)
    {
            cv::Mat imageWithKeypoints;
            cv::drawKeypoints(m_SLAM->GetImageFedToTrackerLeft(), m_SLAM->GetTrackedKeyPointsUn(), imageWithKeypoints, cv::Scalar(0,255,0));
            keypointVisualizationPublisher->publish(*cv_bridge::CvImage(poseMsg.header, "rgb8", imageWithKeypoints).toImageMsg());
    }

    //const auto timeOutputHandlingEnd {std::chrono::steady_clock::now()};
    //const auto durationOutputHandling_ms {std::chrono::duration_cast<std::chrono::milliseconds>(timeOutputHandlingEnd - timeOutputHandlingStart)};


     //RCLCPP_INFO(this->get_logger(), "SLAM estimation took: %ld ms, output handling took: %ld ms", durationSLAMEstimation_ms.count(), durationOutputHandling_ms.count());



}

std::string StereoSlamNode::stampToString(builtin_interfaces::msg::Time stamp) const
{
    std::ostringstream oss;
    oss << std::setw(9) << std::setfill('0') << stamp.nanosec;
    std::string timeInformationStr {std::to_string(stamp.sec) + "_" +  oss.str()};
    return timeInformationStr;
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





