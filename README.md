
===

Build
---

1.Building ORB-SLAM3

    git clone https://github.com/sam051200/ORB_SLAM3.git 
	cd ORB_SLAM3
	chmod +x build.sh 
	./build.sh 
2.Building ORB-SLAM3 for ROS nodes 

    sudo apt install python-is-python3 
  	echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS" >> ~/.bashrc 
	source ~/.bashrc
	chmod +x build_ros.sh 
   	./build_ros.sh
`PATH/ORB_SLAM3/Examples/ROS 請填入的絕對路徑`

使用
---
1.config

修改 `./camera/mvs.yaml`，填入相機參數以及map儲存、讀取路徑

2.run

	跑`rosrun.sh`
只須注意最後兩個參數,分別為`topic 名稱`、`輸入模式` 
要修改的話修改`Examples/ROS/ORB_SLAM3/src/ros_mono.cc`



   
