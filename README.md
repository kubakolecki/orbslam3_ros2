# orbslam3_ros2
Use branch pose_publisher!!!
Refer to [https://github.com/zang09/ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED) on how to build. But:
first you need to build https://github.com/kubakolecki/ros_common_messages.
Only valid for stereo slam mode. Few things were fixed. Works with ROS2 Jazzy on Ubuntu 24.04.
Node publishes pose message so can be used in navigation. It also publishes [GeoreferencedStereoImageMessage](https://github.com/kubakolecki/ros_common_messages/blob/main/msg/GeoreferencedStereoImage.msg).
This message can be used to publish stereorectified images, pose (left camera), camera matrices and extrinsics. This message contains also the ORB-SLAM3 sparse map as a sparse depth. Check the [readme of this package](https://github.com/kubakolecki/ros_common_messages) to learn how the sparse depth information is stored. Exemplary usage: stereorectified image + sparse depth cues can be used to get more accurate neural monocular depth estimation that only in case of models where the input data is a single image. For example see this paper:
[Boosting Monocular Depth Estimation with Lightweight 3D Point Fusion](https://arxiv.org/abs/2012.10296)


## Acknowledgments
This repository is modification of [https://github.com/zang09/ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED), which is modification of [this](https://github.com/curryc/ros2_orbslam3) repository.
Credits to zang09.

