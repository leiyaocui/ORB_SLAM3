Vocabulary=/home/yuyang/dev/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt
Setting=/home/yuyang/dev/SLAM/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_sim.yaml
DATA=/home/yuyang/dev/SLAM/dataset/rosbag2_2023_10_13-17_18_05
ImageAssociation=${DATA}/image_filelist.txt
ImuAssociation=${DATA}/imu_filelist.txt

EXEC=/home/yuyang/dev/SLAM/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_sim

${EXEC} ${Vocabulary} ${Setting} ${ImageAssociation} ${ImuAssociation}
