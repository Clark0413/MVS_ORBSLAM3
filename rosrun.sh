#=================================MANO 
# rosrun ORB_SLAM3 Mono /dpds/ORB_SLAM3/Vocabulary/ORBvoc.txt /dpds/ORB_SLAM3/Examples/ROS/mycam.yaml /image

#---mvs
rosrun ORB_SLAM3 Mono ./Vocabulary/ORBvoc.txt ./camera/mvs.yaml /image_mvs mvs
# rosrun ORB_SLAM3 Mono ./Vocabulary/ORBvoc.txt ./camera/mvs.yaml /image_rgb rgb