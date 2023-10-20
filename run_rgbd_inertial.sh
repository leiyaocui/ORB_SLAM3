Vocabulary=./Vocabulary/ORBvoc.txt
Setting=./Examples/RGB-D-Inertial/rgbd_inertial_sim.yaml
DATA=../dataset/rosbag2_2023_10_13-17_18_05
ImageAssociation=${DATA}/image_filelist.txt
ImuAssociation=${DATA}/imu_filelist.txt

EXEC=./Examples/RGB-D-Inertial/rgbd_inertial_sim

${EXEC} ${Vocabulary} ${Setting} ${ImageAssociation} ${ImuAssociation}
