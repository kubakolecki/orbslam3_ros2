# orbslam3_ros2
**Use branch pose_publisher!!!**
Refer to [https://github.com/zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2) on how to build, because this repository is modification of https://github.com/zang09/ORB_SLAM3_ROS2. But:
first you need to build https://github.com/kubakolecki/ros_common_messages. Also you need to build ORB_SLAM3 first. **You need to build this fork** [ORB-SLAM3-STEREO-FIXED](https://github.com/kubakolecki/ORB-SLAM3-STEREO-FIXED) instead of original version of 
ORB-SLAM3-STEREO-FIXED because in the master branch of my fork I added some getters so the ROS2 node can access data from ORB-SLAM3.
**Only valid for stereo slam mode**. Few things were fixed. Works with ROS2 Jazzy on Ubuntu 24.04.
Node publishes pose message so can be used in navigation. It also publishes [GeoreferencedStereoImageMessage](https://github.com/kubakolecki/ros_common_messages/blob/main/msg/GeoreferencedStereoImage.msg).
This message can be used to publish stereorectified images, pose (left camera), camera matrices and extrinsics. This message contains also the ORB-SLAM3 sparse map as a sparse depth. Check the [readme of this package](https://github.com/kubakolecki/ros_common_messages) to learn how the sparse depth information is stored. Exemplary usage: stereorectified image + sparse depth cues can be used to get more accurate neural monocular depth estimation that only in case of models where the input data is a single image. For example see this paper:
[Boosting Monocular Depth Estimation with Lightweight 3D Point Fusion](https://arxiv.org/abs/2012.10296)


## Acknowledgments
This repository is modification of [https://github.com/zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2).
Credits to zang09.
