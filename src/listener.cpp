#include <ros/ros.h>
#include <std_msgs/String.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  

  //線速度
  double linear_x = msg->twist.twist.linear.x;
  double linear_y = msg->twist.twist.linear.y;
  double linear_z = msg->twist.twist.linear.z;

  //回転速度
  double angular_x = msg->twist.twist.angular.x;
  double angular_y = msg->twist.twist.angular.y;
  double angular_z = msg->twist.twist.angular.z;

  //位置
  double pose_x = msg->pose.pose.position.x;
  double pose_y = msg->pose.pose.position.y;
  double pose_z = msg->pose.pose.position.z;

  ROS_INFO('linear_vel: x=%.3f, y=%.3f, z=%.3f',pose_x,pose_y, pose_z);
  ROS_INFO("Angular_vel: x=%.3f, y=%.3f, z=%.3f", angular_x, angular_y, angular_z);
  ROS_INFO("Position: x=%.3f, y=%.3f, z=%.3f", pose_x, pose_y, pose_z);
} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);

  ros::spin();
  return 0;
}
