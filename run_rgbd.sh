Vocabulary=./Vocabulary/ORBvoc.txt
Setting=./Examples/RGB-D/rgbd_sim.yaml
DATA=../dataset/rosbag2_2023_10_19-20_18_12
ImageAssociation=${DATA}/image_filelist.txt
EXEC=./Examples/RGB-D/rgbd_sim

RESULT=./tf_pred.txt

${EXEC} ${Vocabulary} ${Setting} ${ImageAssociation}

mv ${RESULT} ${DATA}/
