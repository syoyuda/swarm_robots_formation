#include <#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <cmath>



// 仮想ばねマスダンパから受ける力を計算する
geometry_msgs::Vector3 calculateVirtualForce(const geometry_msgs::Point& pos1, const geometry_msgs::Point& pos2,
                                             const geometry_msgs::Twist& vel1, const geometry_msgs::Twist& vel2
                                             double rest_length) //位置、相対速度、ばねの自然長
{
    // ばね定数とダンパ定数
    double spring_constant = 1.0;
    double damping_constant = 0.1;

    //出力する制御力
    geometry_msgs::Vector3 virtual_force;

    // 相対位置ベクトル
    double dx = pos2.x - pos1.x;
    double dy = pos2.y - pos1.y;
    double dz = 0;              //2Dなので無視する

    // 相対距離
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    // 単位ベクトル いらないかも
    geometry_msgs::Vector3 unit_vector;
    if (distance != 0) {
        unit_vector.x = dx / distance;
        unit_vector.y = dy / distance;
        unit_vector.z = dz / distance;
    } else {
        unit_vector.x = unit_vector.y = unit_vector.z = 0;
    }

    // フックの法則
    double spring_force = -spring_constant * (distance - rest_length);

    // ダンパの力
    double damping_force = -damping_constant * (relative_velocity.x * unit_vector.x);

    // 合力
    virtual_force.x = (spring_force + damping_force) * unit_vector.x;
    virtual_force.y = (spring_force + damping_force) * unit_vector.y;
    virtual_force.z = (spring_force + damping_force) * unit_vector.z;

    return virtual_force;
}




// 仮想力から速度指令を生成する関数
geometry_msgs::Twist Force2velocity(const geometry_msgs::Vector3& force, 
                                    const double current_yaw)
{
    geometry_msgs::Twist vel;
}

int main(){
    return 0;
}