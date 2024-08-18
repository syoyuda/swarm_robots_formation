#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <string>
#include <vector>

// ロボットの名簿
std::vector<std::string> robot_names_list = {"robot1", "robot2"};
geometry_msgs::Point robot_positions[2];

// 位置、速度情報の取得
void odomCallback1(const nav_msgs::Odometry::ConstPtr& msg){

  //線速度
  double linear_x = msg->twist.twist.linear.x;
  double linear_y = msg->twist.twist.linear.y;
  double linear_z = msg->twist.twist.linear.z;

  //回転速度
  double angular_x = msg->twist.twist.angular.x;
  double angular_y = msg->twist.twist.angular.y;
  double angular_z = msg->twist.twist.angular.z;

  //位置
  double pos_x = msg->pose.pose.position.x;
  double pos_y = msg->pose.pose.position.y;
  double pos_z = msg->pose.pose.position.z;

  ROS_INFO("linear_vel: x=%.2f, y=%.2f, z=%.2f",linear_x,linear_y,linear_z);
  ROS_INFO("Angular_vel: x=%.2f, y=%.2f, z=%.2f", angular_x, angular_y, angular_z);
  ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f\n", pos_x, pos_y, pos_z);
} 

void odomCallback2(const nav_msgs::Odometry::ConstPtr& msg){

  //線速度
  double linear_x2 = msg->twist.twist.linear.x;
  double linear_y2 = msg->twist.twist.linear.y;
  double linear_z2 = msg->twist.twist.linear.z;

  //回転速度
  double angular_x2 = msg->twist.twist.angular.x;
  double angular_y2 = msg->twist.twist.angular.y;
  double angular_z2 = msg->twist.twist.angular.z;

  //位置
  double pos_x2 = msg->pose.pose.position.x;
  double pos_y2 = msg->pose.pose.position.y;
  double pos_z2 = msg->pose.pose.position.z;

  ROS_INFO("linear_vel: x=%.2f, y=%.2f, z=%.2f",linear_x2,linear_y2,linear_z2);
  ROS_INFO("Angular_vel: x=%.2f, y=%.2f, z=%.2f", angular_x2, angular_y2, angular_z2);
  ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f\n", pos_x2, pos_y2, pos_z2);
} 



// 二点間のユークリッド距離を計算する関数
double calculateDistance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2)
{
    ROS_INFO("robot1 = (%f, %f) robot2 = (%f, %f)",point1.x, point1.y, point2.x, point2.y);
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

int main(int argc, char** argv)
{
    // ROSノードの初期化
    ros::init(argc, argv, "relative_distance_calculator");

    ros::NodeHandle nh;

    // サブスクライバの定義
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1000, modelStatesCallback);

    // ループレートを設定
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // コールバック関数を呼び出す
        ros::spinOnce();

        // ロボット同士の相対距離を計算
        double distance = calculateDistance(robot_positions[0], robot_positions[1]);
        ROS_INFO("Relative distance between %s and %s: %f", robot_names_list[0].c_str(), robot_names_list[1].c_str(), distance);

        loop_rate.sleep();
    }

    return 0;
}
