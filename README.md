ORB-SLAM3 + EQ
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

可參考 `./Examples/ROS/mycam.yaml`，填入相機參數以及map儲存、讀取路徑

2.運行

	rosrun ORB_SLAM3 Mono ./ORB_SLAM3/Vocabulary/ORBvoc.txt ./ORB_SLAM3/Examples/ROS/mycam.yaml /image
`./ORB_SLAM3/Examples/ROS/mycam.yaml` 填入自己相機的設定檔路徑

`/image`填入ROS 影像Topic的名稱

test_bag 
https://mailntustedutw-my.sharepoint.com/:u:/g/personal/m11102142_ms_ntust_edu_tw/EfrhiSP828BGgy1lIW7vcHYBkjEw27qrw3CGHZCZ2BEakg?e=ruXtEa

3.結束運行

	rosnode kill -a
	
結束後會儲存map



   
