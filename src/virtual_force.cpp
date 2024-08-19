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
    // ばねの定数とダンパ定数
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

    // 単位ベクトル
    geometry_msgs::Vector3 unit_vector;
    if (distance != 0) {
        unit_vector.x = dx / distance;
        unit_vector.y = dy / distance;
        unit_vector.z = dz / distance;
    } else {
        unit_vector.x = unit_vector.y = unit_vector.z = 0;
    }

    // ばねの力（フックの法則に基づく）
    double spring_force = -spring_constant * (distance - rest_length);

    // ダンパの力（相対速度を使用）
    double damping_force = -damping_constant * (relative_velocity.x * unit_vector.x);

    // 合力
    virtual_force.x = (spring_force + damping_force) * unit_vector.x;
    virtual_force.y = (spring_force + damping_force) * unit_vector.y;
    virtual_force.z = (spring_force + damping_force) * unit_vector.z;

    return virtual_force;
}




// 仮想力から実際の速度指令を生成する関数
geometry_msgs::Twist Force2velocity(const geometry_msgs::Vector3& force, 
                                              const double current_yaw)
{
    geometry_msgs::Twist cmd_vel;


    // 定数の定義
    const double max_linear_speed = 0.5; // 最大線速度 [m/s]
    const double max_angular_speed = 1.0; // 最大角速度 [rad/s]
    const double k_alpha = 1.0; // 角速度制御の比例ゲイン
    const double distance_threshold = 0.1; // 停止するための距離閾値 [m]

    // 仮想力の目標方向（ポテンシャル法に基づく）
    double desired_yaw = atan2(force.y, force.x);

    // 現在のロボットの向きと目標方向との差
    double yaw_difference = desired_yaw - current_yaw;

    // 角度エラーを[-pi, pi]の範囲に正規化
    if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    if (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    // 角速度の計算（yawエラーに基づくPD制御）
    cmd_vel.angular.z = k_alpha * yaw_error;

    // 角速度を制限
    if (cmd_vel.angular.z > max_angular_speed) cmd_vel.angular.z = max_angular_speed;
    if (cmd_vel.angular.z < -max_angular_speed) cmd_vel.angular.z = -max_angular_speed;

    // 線速度の計算
    double force_magnitude = sqrt(force.x * force.x + force.y * force.y);

    // 距離が閾値以上の場合にのみ前進
    if (force_magnitude > distance_threshold)
    {
        cmd_vel.linear.x = max_linear_speed;
    }
    else
    {
        cmd_vel.linear.x = 0.0;
    }

    return cmd_vel;
}

int main(){
    return 0;
}