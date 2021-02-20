#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopCerus
{
public:
  TeleopCerus();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh;

  int linear, angular, strafe;
  double l_scale, a_scale, s_scale;
  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;

};


TeleopCerus::TeleopCerus():
  linear(1),
  angular(2),
  strafe(3)
{

  nh.param("axis_linear", linear, linear);
  nh.param("axis_angular", angular, angular);
  nh.param("axis_strafe", strafe, strafe);
  nh.param("scale_angular", a_scale, a_scale);
  nh.param("scale_linear", l_scale, l_scale);
  nh.param("scale_strafe", s_scale, s_scale);


  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopCerus::joyCallback, this);

}

void TeleopCerus::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale*joy->axes[angular];
  twist.linear.x = l_scale*joy->axes[linear];  
  twist.linear.y = s_scale*joy->axes[strafe];
  vel_pub.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_cerus");
  TeleopCerus teleop_cerus;

  ros::spin();
}

