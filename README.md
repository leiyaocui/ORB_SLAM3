# ORB-SLAM3

This repo provides a modified ORB-SLAM3 for our RGBD SLAM demand. The details of installation and usage refers to [README_ORBSLAM3](https://github.com/leiyaocui/ORB_SLAM3/blob/master/README_ORBSLAM3.md).

## Dependencies

Before building this project, you need to install these dependencies first:

- [g2o](https://github.com/RainerKuemmerle/g2o) dcf7935

We provide a dataloader that loads data from local path and a toolkit. The toolkit contains scripts for rosbag extraction and TSDF reconstruction. If you try to use it, you need to install some python libraries first: rosbags, open3d, numpy, scipy, and rclpy.