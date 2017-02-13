#include "publisher.h"

#ifdef HAS_ROS

#include <tf2_eigen/tf2_eigen.h>

class ORBSLAM2Publisher;

inline bool ros_init(const std::string & name)
{
  int argc = 0;
  char * argv[] = {0};
  ros::init(argc, argv, name.c_str());
  if(!ros::master::check())
  {
    std::cerr << "ROS master is not available, continue without ROS functionalities";
    return false;
  }
  return true;
}

struct ROSBridgeImpl
{
  ROSBridgeImpl() : ros_is_init(ros_init("orb_slam2")),
    nh( ros_is_init ? new ros::NodeHandle() : 0 )
    {
    }

  void update_pose(const Eigen::Affine3d& pose);

  bool ros_is_init;
  std::shared_ptr<ros::NodeHandle> nh;
  std::shared_ptr<ORBSLAM2Publisher> pub;
};

std::unique_ptr<ROSBridgeImpl> ROSBridge::impl = std::unique_ptr<ROSBridgeImpl>(new ROSBridgeImpl());

std::shared_ptr<ros::NodeHandle> ROSBridge::get_node_handle()
{
  return impl->nh;
}

void ROSBridge::update_pose(const Eigen::Affine3d& pose)
{
  // Publish pose
  impl->update_pose(pose);
}

void ROSBridgeImpl::update_pose(const Eigen::Affine3d& pose)
{
  if(pub == nullptr)
  {
    pub.reset(new ORBSLAM2Publisher("orb_slam2", 100));
  }
  pub->update_pose(pose);
}



ORBSLAM2Publisher::ORBSLAM2Publisher(const std::string& prefix, unsigned int rate)
  :
    nh(ROSBridge::get_node_handle()),
    prefix(prefix), rate(rate),
    tf_caster(),
    running(true),
    th(std::bind(&ORBSLAM2Publisher::publishThread, this))
{

};

ORBSLAM2Publisher::~ORBSLAM2Publisher()
{
  running = false;
  th.join();
}

void ORBSLAM2Publisher::update_pose(const Eigen::Affine3d& pose)
{
  geometry_msgs::TransformStamped msg = tf2::eigenToTransform(pose);
  msg.header.frame_id = "map";
  msg.child_frame_id = prefix+"/pose";
  mut.lock();
  tf = msg;
  mut.unlock();
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



#else

namespace ros
{
  class NodeHandle {};
}

struct ROSBridgeImpl
{
  ROSBridgeImpl() : nh(0) {}
  std::shared_ptr<ros::NodeHandle> nh;
};

std::unique_ptr<ROSBridgeImpl> ROSBridge::impl = std::unique_ptr<ROSBridgeImpl>(new ROSBridgeImpl());

std::shared_ptr<ros::NodeHandle> ROSBridge::get_node_handle()
{
  return impl->nh;
}

void ROSBridge::update_pose(const Eigen::Affine3d& pose)
{
}

void ROSBridge::shutdown()
{
}

#endif

