#include "publisher.h"

#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/eigen.hpp>
#include "../../../include/MapPoint.h"
#include "../../../include/System.h"

using ORB_SLAM2::MapPoint;

ORBSLAM2Publisher::ORBSLAM2Publisher(ORB_SLAM2::System* pSLAM, std::shared_ptr<ros::NodeHandle> nh, const std::string& prefix, unsigned int rate)
  :
    pSLAM_(pSLAM),
    nh(nh),
    prefix(prefix), rate(rate),
    tf_caster(),
    running(true),
    th(std::bind(&ORBSLAM2Publisher::publishThread, this))
{
  cloud_pub = nh->advertise<PCLCloud>(prefix+"/cloud", 1);
  pose_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(prefix+"/rgb_pose", 1000);
  reset_service = nh->advertiseService(prefix+"/reset", &ORBSLAM2Publisher::reset_callback, this);
};

ORBSLAM2Publisher::~ORBSLAM2Publisher()
{
  running = false;
  th.join();
}

void ORBSLAM2Publisher::update_pose(const Eigen::Affine3d& p)
{
  Eigen::Affine3d pose = p.inverse();
  // Convert from right-handed Z forward (ORB_SLAM2) to
  // right-handed X forward (ROS)

  // Mapping ROS ORB_SLAM2
  //          +x  +z
  //          +y  -x
  //          +z  -y
  Eigen::Matrix3d R_toROS = Eigen::Matrix3d::Zero();
  R_toROS(2,0) = 1;
  R_toROS(0,1) = -1;
  R_toROS(1,2) = -1;
  Eigen::Matrix4d pose_ros = Eigen::Matrix4d::Identity();
  // First transform rotation matrix such that rotation is applied in
  // ROS frame convention.
  // Then rotate the coordinate system from Z forward to X forward axes
  pose_ros.block<3,3>(0,0) = R_toROS.inverse() * pose.matrix().block<3,3>(0,0) * R_toROS;
  // Apply the translation with X-forward convention
  pose_ros.block<3,1>(0,3) << pose(2,3), -pose(0,3), -pose(1,3);
  Eigen::Affine3d pose_ros_affine(pose_ros);
  // std::cout << "Pose ROS (x forward, y left, z up): \n" << pose_ros << std::endl;
  auto transform = tf2::eigenToTransform(pose_ros_affine);
  transform.header.frame_id = "map";
  transform.child_frame_id = prefix+"/pose_rgbd";
  tf_caster.sendTransform(transform);

  // PUBLISH AS PoseWithCovarianceStamped
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  tf::poseEigenToMsg(pose_ros_affine, msg.pose.pose);
  // XXX fill covariance
  pose_pub.publish(msg);
}

void ORBSLAM2Publisher::update_tracked_map()
{
  const auto& map_points = pSLAM_->GetAllMapPoints();

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


bool ORBSLAM2Publisher::reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cout << "Reset service called, resetting ORB_SLAM2" << std::endl;
  // RESET
  pSLAM_->Reset();
  return true;
}

void ORBSLAM2Publisher::publishThread()
{
  ros::Rate rt(rate);
  while(running && ros::ok())
  {
    ros::spinOnce();
    rt.sleep();
  }
}
