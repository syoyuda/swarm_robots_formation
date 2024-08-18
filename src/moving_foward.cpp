#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//毎秒0.1mで1m前進させるプログラム


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moving_foward");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::WallTime wall_begin = ros::WallTime::now();
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    //10秒かけて1m移動する
    geometry_msgs::Twist move_cmd;
    double velocity = 0.1;
    ros::WallTime wall_now = ros::WallTime::now();
    ros::WallDuration wall_duration = wall_now - wall_begin;
    move_cmd.linear.x = velocity;
    chatter_pub.publish(move_cmd);

    if (wall_duration > ros::WallDuration(10))
    {
      move_cmd.linear.x = 0;
      chatter_pub.publish(move_cmd);
    }

    ROS_INFO("linear.x: %.2f", move_cmd.linear.x);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
