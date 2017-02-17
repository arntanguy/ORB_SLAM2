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

  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  Eigen::Matrix3d rr;
  rr << 0, -1, 0,
        0, 0, -1,
        1, 0, 0;

  // Convert to NED (X forward, Y left, Z up)
  // ORB has z forward, -x left, -y up
  p.matrix().block<3,3>(0,0) = Rwc * rr; // r_xfwd;
  p.matrix().block<3,1>(0,3) = twc; // trans;

  // Rotate to ROS map
  // Rotate -pi/2 around x, then pi/2 around y
  Eigen::Matrix3d m;
  m =
      Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0.5*M_PI,  Eigen::Vector3d::UnitY());
  Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
  M.block<3,3>(0,0) = m;

  // Rotate whole cordinate system to match with ROS map
  p.matrix() = M * p.matrix();

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
