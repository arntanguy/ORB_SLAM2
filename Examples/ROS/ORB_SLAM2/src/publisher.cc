#include "publisher.h"

#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/core/eigen.hpp>
#include "../../../include/MapPoint.h"

using ORB_SLAM2::MapPoint;

ORBSLAM2Publisher::ORBSLAM2Publisher(std::shared_ptr<ros::NodeHandle> nh, const std::string& prefix, unsigned int rate)
  :
    nh(nh),
    prefix(prefix), rate(rate),
    tf_caster(),
    running(true),
    th(std::bind(&ORBSLAM2Publisher::publishThread, this))
{
  cloud_pub = nh->advertise<PCLCloud>(prefix+"/cloud", 1);
};

ORBSLAM2Publisher::~ORBSLAM2Publisher()
{
  running = false;
  th.join();
}

void ORBSLAM2Publisher::update_pose(const Eigen::Affine3d& pose)
{
  Eigen::Affine3d pose_inv = pose.inverse();
  const Eigen::Matrix3d& Rwc = pose_inv.rotation(); //Rcw.transpose();
  const Eigen::Vector3d& twc = pose_inv.translation(); //-Rwc * tcw;

  // Matrix to flip sign of one of the z axis
  Eigen::Matrix3d Sz = Eigen::Matrix3d::Identity();
  Sz(2,2) = -1;

  // Convert rotation to right-handed coordinate system
  Eigen::Matrix3d rh = Sz * Rwc * Sz;
  // Rotation axes
  // ROS (x,y,z) -> ORB (z,x,y)
  Eigen::Matrix3d rot;
  rot << 0,1,0,
      0,0,1,
      1,0,0;
  // Swap axes so that we have x forward, and y left
  // How does this work exactly?
  Eigen::Matrix3d r_xfwd = rot.inverse() * rh * rot;

  // Remap translations to ROS frame
  Eigen::Vector3d trans;
  trans(0) = twc(2);
  trans(1) = -twc(0);
  trans(2) = -twc(1);

  // Set transformed pose
  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  p.matrix().block<3,3>(0,0) = r_xfwd;
  p.matrix().block<3,1>(0,3) = trans;

  geometry_msgs::TransformStamped msg = tf2::eigenToTransform(p);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.child_frame_id = prefix+"/pose";
  mut.lock();
  tf = msg;
  mut.unlock();
}

void ORBSLAM2Publisher::update_tracked_map(const std::vector<MapPoint*>& map_points)
{
  static size_t prev_size = 0;
  if(map_points.size() != prev_size)
  {
    PCLCloud cloud;
    cloud.header.frame_id = "map";
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);

    for(MapPoint* p : map_points)
    {
      if(p != nullptr)
      {
        Eigen::Matrix4f pose;
        cv::cv2eigen(p->GetWorldPos(), pose);
        Eigen::Vector3f world_trans = pose.block<3,1>(0,0);
        pcl::PointXYZ pcl_point;
        pcl_point.x = world_trans(2);
        pcl_point.y = -world_trans(0);
        pcl_point.z = -world_trans(1);
        cloud.points.push_back(pcl_point);
      }
    }
    cloud.height = 1;
    cloud.width = cloud.size();
    cloud_pub.publish(cloud);
    prev_size = cloud.size();
  }
}

void ORBSLAM2Publisher::publishThread()
{
  ros::Rate rt(rate);
  while(running && ros::ok())
  {
    if(mut.try_lock())
    {
      try
      {
        tf_caster.sendTransform(tf);
      }
      catch(const ros::serialization::StreamOverrunException & e)
      {
        std::cerr << "EXCEPTION WHILE PUBLISHING STATE: " << e.what() << std::endl;
      }
      mut.unlock();
    }
    rt.sleep();
  }
}
