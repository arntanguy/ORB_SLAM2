/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* Copyright (C) 2017 Arnaud TANGUY <tanguy at i3s dot unice dot fr> (University of Nice Sophia Antipolis)
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

#pragma once

#include <memory>
#include <vector>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <mutex>
#include <memory>

class ORBSLAM2Publisher
{
 public:
  ORBSLAM2Publisher(std::shared_ptr<ros::NodeHandle> nh, const std::string& prefix, unsigned int rate);
  ~ORBSLAM2Publisher();

  void update_pose(const Eigen::Affine3d& pose);

 private:
  void publishThread();

 private:
  std::shared_ptr<ros::NodeHandle> nh;
  std::string prefix;
  unsigned int rate;
  tf2_ros::TransformBroadcaster tf_caster;
  bool running = false;
  std::mutex mut;
  std::thread th;
  geometry_msgs::TransformStamped tf;
};

