#include <algorithm>
#include <assert.h>

#include <rotors_gazebo_plugins/gazebo_tf_firefly_plugin.hpp>

#if (GAZEBO_MAJOR_VERSION < 8)
#include <gazebo/math/gzmath.hh>
#endif
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <gazebo/gazebo_config.h>

namespace gazebo {

  GazeboRosTfFirefly::GazeboRosTfFirefly() {}

  // Destructor
  GazeboRosTfFirefly::~GazeboRosTfFirefly() {
    delete rosnode_;
    delete transform_broadcaster_;
  }

  // Load the controller
  void GazeboRosTfFirefly::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();
    //Take the namespace
    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("GazeboRosTfFirefly Plugin missing <robotNamespace>, defaults to \"%s\"", 
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ = 
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }
    //Take the rate
    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("GazeboRosTfFirefly Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    #if (GAZEBO_MAJOR_VERSION >= 8)
        last_update_time_ = this->world->GetSimTime();
    #else
        last_update_time_ = this->world->GetSimTime();
    #endif

    alive_ = true;


    //Get Links DIREFLY DRONE
    b_l_ffd_ = this->parent->GetLink(robot_namespace_+"base_link");
    r0_ffd_ = this->parent->GetLink(robot_namespace_+"rotor_0");
    r1_ffd_ = this->parent->GetLink(robot_namespace_+"rotor_1");
    r2_ffd_ = this->parent->GetLink(robot_namespace_+"rotor_2");
    r3_ffd_ = this->parent->GetLink(robot_namespace_+"rotor_3");
    r4_ffd_ = this->parent->GetLink(robot_namespace_+"rotor_4");
    r5_ffd_ = this->parent->GetLink(robot_namespace_+"rotor_5");

   

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO("Starting GazeboRosTfFirefly Plugin (ns = %s)!", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // start custom queue for diff drive
    this->callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosTfFirefly::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosTfFirefly::UpdateChild, this));
  }


  // Update the controller
  void GazeboRosTfFirefly::UpdateChild() {
    #if (GAZEBO_MAJOR_VERSION >= 8)
        common::Time current_time = this->world->GetSimTime();
    #else
        common::Time current_time = this->world->GetSimTime();
    #endif
        double seconds_since_last_update = (current_time - last_update_time_).Double();

    if (seconds_since_last_update > update_period_) {
       
      tfBaseLink();
      last_update_time_+= common::Time(update_period_);
    }
  }


  // Finalize the controller
  void GazeboRosTfFirefly::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }


  void GazeboRosTfFirefly::QueueThread() 
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }


  void GazeboRosTfFirefly::tfBaseLink(void)
  {
    static tf::TransformBroadcaster br;
    math::Pose  tf_r0_ffd_,tf_r1_ffd_,tf_r2_ffd_,tf_r3_ffd_,tf_r4_ffd_,tf_r5_ffd_;

    //tf to ARDRONE
    tf_r0_ffd_ = r0_ffd_ -> GetWorldCoGPose(); 
    tf_r1_ffd_ = r1_ffd_ -> GetWorldCoGPose();
    tf_r2_ffd_ = r2_ffd_ -> GetWorldCoGPose();
    tf_r3_ffd_ = r3_ffd_ -> GetWorldCoGPose();
    tf_r4_ffd_ = r4_ffd_ -> GetWorldCoGPose();
    tf_r5_ffd_ = r5_ffd_ -> GetWorldCoGPose();
    

    //Transform for ARDRONE
    tf::Transform t_r0_,t_r1_, t_r2_, t_r3_, t_r4_, t_r5_, t_imu_, t_imugt_, t_odom_, t_odomgt_, t_ine_;
   
    t_r0_.setOrigin( tf::Vector3(0.186195, 0.1075, 0.037) );
    t_r0_.setRotation(tf::Quaternion(0,0,tf_r0_ffd_.rot.z,tf_r0_ffd_.rot.w));

    t_r1_.setOrigin( tf::Vector3(0.0, 0.215, 0.037) );
    t_r1_.setRotation(tf::Quaternion(0,0,tf_r1_ffd_.rot.z,tf_r1_ffd_.rot.w));

    t_r2_.setOrigin( tf::Vector3(-0.186195, 0.1075, 0.037) );
    t_r2_.setRotation(tf::Quaternion(0,0,tf_r2_ffd_.rot.z,tf_r2_ffd_.rot.w));

    t_r3_.setOrigin( tf::Vector3(-0.186195, -0.1075, 0.037) );
    t_r3_.setRotation(tf::Quaternion(0,0,tf_r3_ffd_.rot.z,tf_r3_ffd_.rot.w));

    t_r4_.setOrigin( tf::Vector3(0.0, -0.215, 0.037) );
    t_r4_.setRotation(tf::Quaternion(0,0,tf_r4_ffd_.rot.z,tf_r4_ffd_.rot.w));  

    t_r5_.setOrigin( tf::Vector3(0.186195, -0.1075, 0.037) );
    t_r5_.setRotation(tf::Quaternion(0,0,tf_r5_ffd_.rot.z,tf_r5_ffd_.rot.w));    

    t_imu_.setOrigin( tf::Vector3(0, 0, 0) );
    t_imu_.setRotation(tf::Quaternion(0,0,0,1));
    t_imugt_.setOrigin( tf::Vector3(0, 0, 0) );
    t_imugt_.setRotation(tf::Quaternion(0,0,0,1));

    t_odom_.setOrigin( tf::Vector3(0, 0, 0) );
    t_odom_.setRotation(tf::Quaternion(0,0,0,1));
    t_odomgt_.setOrigin( tf::Vector3(0, 0, 0) );
    t_odomgt_.setRotation(tf::Quaternion(0,0,0,1));

    t_ine_.setOrigin( tf::Vector3(0, 0, 0) );
    t_ine_.setRotation(tf::Quaternion(0,0,0,1));
    

    //Broadcaster TF ARDRONE
    br.sendTransform(tf::StampedTransform(t_r0_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"rotor_0"));
    br.sendTransform(tf::StampedTransform(t_r1_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"rotor_1"));
    br.sendTransform(tf::StampedTransform(t_r2_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"rotor_2"));
    br.sendTransform(tf::StampedTransform(t_r3_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"rotor_3"));
    br.sendTransform(tf::StampedTransform(t_r4_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"rotor_4"));
    br.sendTransform(tf::StampedTransform(t_r5_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"rotor_5"));
    br.sendTransform(tf::StampedTransform(t_imu_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"imu_link"));
    br.sendTransform(tf::StampedTransform(t_imugt_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"imugt_link"));
    br.sendTransform(tf::StampedTransform(t_odom_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"odometry_sensor1_link"));
    br.sendTransform(tf::StampedTransform(t_odomgt_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"odometry_sensorgt_link"));
    br.sendTransform(tf::StampedTransform(t_ine_, ros::Time::now(), robot_namespace_+"base_link",robot_namespace_+"base_link_inertia"));
  }


  GZ_REGISTER_MODEL_PLUGIN(GazeboRosTfFirefly)

}