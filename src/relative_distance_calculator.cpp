#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <string>
#include <vector>

// ロボットの名簿
std::vector<std::string> robot_names_list = {"robot1", "robot2"};
geometry_msgs::Point robot_positions[2];

// 位置情報の取得
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        for (size_t j = 0; j < robot_names_list.size(); ++j)
        {
            
            if (msg->name[i] == robot_names_list[j])
            {
                robot_positions[j] = msg->pose[i].position;
            }
        }
    }
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
