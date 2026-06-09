#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ros_common_messages/msg/georeferenced_stereo_image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "cv_bridge/cv_bridge.hpp"

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

#include <Eigen/Core>

#include <vector>

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify);

    ~StereoSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    ORB_SLAM3::System* m_SLAM;

    bool doRectify;
    cv::Mat M1l,M2l,M1r,M2r;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    //cv::Mat previousLeftImage;
    //cv::Mat previousRightImage;

    //std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > left_sub;
    //std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > right_sub;

    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher;
    rclcpp::Publisher<ros_common_messages::msg::GeoreferencedStereoImage>::SharedPtr georeferencedStereoPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereoRectifiedLeftPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereoRectifiedRightPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr keypointVisualizationPublisher;

    nav_msgs::msg::Path pathMsg;

    bool doPublishStereorectifiedImages{false};
    bool doWritePosesToTextFile{false};
    bool doPublishTrackedKeypointsVisualization{false};
    bool doSaveLocalMapToFile{false};
    std::string pathToSaveLocalMap{};
    std::string pathToSavePoses{};
    std::ofstream posesOutputFile;

    //float timeToWaintToPublishStereoImageInSeconds{1.0f};
    //uint64_t lastTimePublishedStereoImage{0};
    //bool hasFirstPoseBeenPublished{false};
    float distanceThresholdToPublishStereoImage{0.25f}; //meters

    Eigen::Vector3f lastPublishedPosition{Eigen::Vector3f::Zero()};

    std::string stampToString(builtin_interfaces::msg::Time stamp) const;
    void writeMapPointsToFile(const std::vector<ORB_SLAM3::MapPoint*>& mapPoints, const std::vector<cv::KeyPoint>& keyPoints,const cv::Mat& image, const vector<double>& depths);

};

#endif
