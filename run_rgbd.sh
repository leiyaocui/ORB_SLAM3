Vocabulary=/home/yuyang/dev/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt
Setting=/home/yuyang/dev/SLAM/ORB_SLAM3/Examples/RGB-D/rgbd_sim.yaml
DATA=/home/yuyang/dev/SLAM/dataset/rosbag2_2023_10_19-20_18_12
ImageAssociation=${DATA}/image_filelist.txt
EXEC=/home/yuyang/dev/SLAM/ORB_SLAM3/Examples/RGB-D/rgbd_sim

RESULT=/home/yuyang/dev/SLAM/ORB_SLAM3/tf_pred.txt

${EXEC} ${Vocabulary} ${Setting} ${ImageAssociation}

mv ${RESULT} ${DATA}/
