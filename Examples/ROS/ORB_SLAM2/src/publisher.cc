#include "publisher.h"
#include <tf2_eigen/tf2_eigen.h>

class ORBSLAM2Publisher;

ORBSLAM2Publisher::ORBSLAM2Publisher(std::shared_ptr<ros::NodeHandle> nh, const std::string& prefix, unsigned int rate)
  :
    nh(nh),
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
  msg.header.stamp = ros::Time::now();
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
