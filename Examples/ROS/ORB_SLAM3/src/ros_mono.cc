/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage_rgb(const sensor_msgs::ImageConstPtr& msg);
    void GrabImage_gray(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings topic_name rgb" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    // ros::Subscriber sub = nodeHandler.subscribe("/mono/left", 1, &ImageGrabber::GrabImage, &igb);
    //  ros::Subscriber sub = nodeHandler.subscribe(argv[3], 1, &ImageGrabber::GrabImage_gray, &igb);
    string a = argv[4];
    ros::Subscriber sub;
    if (a == "mvs"){
        cout<< "mvs_image\n";
        sub = nodeHandler.subscribe(argv[3], 1, &ImageGrabber::GrabImage_gray, &igb);
    }
    else{
        
        cout<< "rgb_image\n";
        sub = nodeHandler.subscribe(argv[3], 1, &ImageGrabber::GrabImage_rgb, &igb);
    }
    // ros::Subscriber sub = nodeHandler.subscribe("/cam0/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryEuRoC("results_tra.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage_gray(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {   
        // cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings:: BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat im = cv_ptr->image;
    cv::cvtColor(im, im, cv::COLOR_BGR2YUV);
    cv::extractChannel(im, im, 0); 
    clock_t start,end;
    start = clock();
    
    mpSLAM->TrackMonocular(im,cv_ptr->header.stamp.toSec());
    end = clock();
    ofstream ofs;
    ofs.open("results/fps.txt", ios::app);
    ofs  <<""<<  double(CLOCKS_PER_SEC/double(end-start))<<endl;
}
void ImageGrabber::GrabImage_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {   
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings:: BGR8);
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return; 
    }
    cv::Mat im = cv_ptr->image;
    mpSLAM->TrackMonocular(im,cv_ptr->header.stamp.toSec());
}


