/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "publisher.h"
#include <opencv2/core/eigen.hpp>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(std::shared_ptr<ros::NodeHandle> nh, ORB_SLAM2::System* pSLAM)
        : mpSLAM(pSLAM), rgb_sub(*nh, "/camera/rgb/image_raw", 1),
        depth_sub(*nh, "camera/depth_registered/image_raw", 1),
        sync(sync_pol(10), rgb_sub,depth_sub),
        pub(nh, "/orb_slam2", 100)
  {
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, this,_1,_2));
  }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync;

    ORBSLAM2Publisher pub;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2");
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }
    auto nh = std::make_shared<ros::NodeHandle>();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(nh, &SLAM);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    // If tracking suceeded
    if(!pose.empty())
    {
      Eigen::Matrix4f b_inv = Eigen::Matrix4f::Identity();
      cv2eigen(pose.inv(),b_inv);

      // Eigen::Matrix4f b_inv = b.inverse();
      Eigen::Matrix3f Rwc = b_inv.block<3,3>(0,0); //Rcw.transpose();
      Eigen::Vector3f twc = b_inv.block<3,1>(0,3); //-Rwc * tcw;

      // Matrix to flip sign of one of the z axis
      Eigen::Matrix3f Sz = Eigen::Matrix3f::Identity();
      Sz(2,2) = -1;

      // Convert rotation to right-handed coordinate system
      Eigen::Matrix3f rh = Sz * Rwc * Sz;
      // Rotation axes
      // ROS (x,y,z) -> ORB (z,x,y)
      Eigen::Matrix3f rot;
      rot << 0,1,0,
          0,0,1,
          1,0,0;
      // Swap axes so that we have x forward, and y left
      // How does this work exactly?
      Eigen::Matrix3f r_xfwd = rot.inverse() * rh * rot;

      // Remap translations to ROS frame
      Eigen::Vector3f trans;
      trans(0) = twc(2);
      trans(1) = -twc(0);
      trans(2) = -twc(1);
      // std::cout << "trans: " << trans << std::endl;

      // Set transformed pose
      Eigen::Matrix4f p = Eigen::Matrix4f::Identity();
      p.block<3,3>(0,0) = r_xfwd;
      p.block<3,1>(0,3) = trans;
      pub.update_pose(Eigen::Affine3d(p.cast<double>()));
    }
}


