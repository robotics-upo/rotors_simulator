#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <iostream>



class Joy {

 private:
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;

  sensor_msgs::Joy control_msg_;
  

  // double current_yaw_vel_;


  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void Publish();

 public:
  Joy();

  int axes_vel_x;
  int axes_vel_y;
  int axes_vel_z;
  int axes_yaw_rate;
  int vel_x_direction;
  int vel_y_direction;
  int vel_z_direction;
  int yaw_rate_direction;

  // int buttons_yaw_left;
  // int buttons_yaw_right;

  double max_vel_x;
  double max_vel_y;
  double max_vel_z;
  double max_yaw_rate;

};

Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<sensor_msgs::Joy> (
    "/dji_sdk/flight_control_setpoint_generic", 10);

  control_msg_.axes.resize(4);
  control_msg_.axes[0] = 0.0;
  control_msg_.axes[1] = 0.0;
  control_msg_.axes[2] = 0.0;
  control_msg_.axes[3] = 0.0; //yaw_rate
  
  //como son axes_ y max_ y buttons_??? -> son struct, creo k en mi caso podria ponerlo como variables sueltas
  pnh.param("axis_vel_y_", axes_vel_y, 2); //horizontal joystick right
  pnh.param("axis_vel_x_", axes_vel_x, 3); //vertical joystick right
  pnh.param("axis_vel_z_", axes_vel_z, 1); //vertical joystick left
  pnh.param("axes_yaw_rate_", axes_yaw_rate, 0); //horizontal joystick right

  pnh.param("axis_direction_vel_y", vel_y_direction, 1); //para invertir control 
  pnh.param("axis_direction_vel_x", vel_x_direction, 1);
  pnh.param("axis_direction_vel_z", vel_z_direction, 1);
  pnh.param("yaw_rate_direction", yaw_rate_direction, 1);

  pnh.param("max_vel_y", max_vel_y, 2.0);  // [m/s]
  pnh.param("max_vel_x", max_vel_x, 2.0);  // [m/s]
  pnh.param("max_yaw_rate", max_yaw_rate, 45.0 * M_PI / 180.0);  // [rad/s] se supone que es 0.5 pero le pongo más para simulacion
  pnh.param("max_vel_z", max_vel_z, 1.0);  // [m/s]
  
  // pnh.param("button_yaw_left_", buttons_yaw_left, 4); //L1
  // pnh.param("button_yaw_right_", buttons_yaw_right, 5); //R1

  namespace_ = nh_.getNamespace(); //esto para que se usa??
  joy_sub_ = nh_.subscribe("joy", 1, &Joy::JoyCallback, this); //este topic debo cambiarlo para meter mas de un joystick o va con namespace??

}


void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  
  control_msg_.axes[1] = msg->axes[axes_vel_y] * max_vel_y * vel_y_direction;
  control_msg_.axes[0] = msg->axes[axes_vel_x] * max_vel_x * vel_x_direction;
  

  // if (msg->buttons[buttons_yaw_left]) {
  //   current_yaw_vel_ = max_yaw_rate;
  // }
  // else if (msg->buttons[buttons_yaw_right]) {
  //   current_yaw_vel_ = -max_yaw_rate;
  // }
  // else {
  //   current_yaw_vel_ = 0.0;
  // }
  // control_msg_.axes[3] = current_yaw_vel_;

  control_msg_.axes[3] = msg->axes[axes_yaw_rate] * max_yaw_rate * yaw_rate_direction; //yaw_rate using sticks

  control_msg_.axes[2] = msg->axes[axes_vel_z] * max_vel_z * vel_z_direction;

  // std::cout<< "La vel target es: " << control_msg_.axes[2] << std::endl;
  
  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "joy_vel_frame";
  Publish();

}

void Joy::Publish() {
  ctrl_pub_.publish(control_msg_); //en vel publicamos un joy que lo tomara nuestro nodo de PID en vel
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_vel");
  Joy joy;

  ros::spin();

  return 0;
}
