#=================================MANO 
# rosrun ORB_SLAM3 Mono /dpds/ORB_SLAM3/Vocabulary/ORBvoc.txt /dpds/ORB_SLAM3/Examples/ROS/mycam.yaml /image

#---mvs
# rosrun ORB_SLAM3 Mono /dpds/ORB_SLAM3/Vocabulary/ORBvoc.txt /dpds/ORB_SLAM3/camera/mvs.yaml /image gray
rosrun ORB_SLAM3 Mono /dpds/ORB_SLAM3/Vocabulary/ORBvoc.txt /dpds/ORB_SLAM3/camera/mvs.yaml /image_edge gray
# rosrun ORB_SLAM3 Mono /dpds/ORB_SLAM3/Vocabulary/ORBvoc.txt /dpds/ORB_SLAM3/camera/mvs.yaml /image_rgb rgb