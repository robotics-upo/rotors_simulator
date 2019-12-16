#ifndef PLUGIN_TF_FIREFLY_HH
#define PLUGIN_TF_FIREFLY_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosTfFirefly : public ModelPlugin {

    public:
      GazeboRosTfFirefly();
      ~GazeboRosTfFirefly();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);
      void getWheelVelocities();
      void tfBaseLink();

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      physics::LinkPtr  b_l_ffd_, r0_ffd_, r1_ffd_, r2_ffd_, r3_ffd_, r4_ffd_, r5_ffd_;

      // ROS STUFF
      ros::NodeHandle* rosnode_;
      tf::TransformBroadcaster *transform_broadcaster_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string init_link_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // DiffDrive stuff
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

  };

}

#endif