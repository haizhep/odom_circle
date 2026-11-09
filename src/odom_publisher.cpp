// ============================================================
// odom_circle: 匀速圆周运动的里程计（正逆时针交替）
// 发布 /odom (nav_msgs/Odometry) 20Hz，并广播 TF odom -> base_link
// ============================================================

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

constexpr double PI = 3.14159265358979323846;

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher() : Node("odom_publisher")
  {
    // 参数：运行时可以改半径和角速度
    radius_ = this->declare_parameter<double>("radius", 1.0);
    omega_ = this->declare_parameter<double>("angular_speed", 0.5);

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_update_ = std::chrono::steady_clock::now();
    timer_ = this->create_wall_timer(
      50ms, std::bind(&OdomPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "odom_circle started: radius=%.2f m, angular_speed=%.2f rad/s",
                radius_, omega_);
  }

private:
  void timer_callback()
  {
    // 1. 用真实经过时间更新角度，避免定时器抖动导致速度误差
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_update_).count();
    last_update_ = now;

    theta_ += omega_ * dt;

    // 2. 走完一整圈就把角速度取反，实现正逆时针交替
    if (theta_ >= 2.0 * PI) {
      theta_ = 2.0 * PI;
      omega_ = -omega_;
    } else if (theta_ <= 0.0) {
      theta_ = 0.0;
      omega_ = -omega_;
    }

    // 3. 圆周上的位置与切线朝向
    const double x = radius_ * std::cos(theta_);
    const double y = radius_ * std::sin(theta_);
    const double yaw = theta_ + (omega_ > 0.0 ? PI / 2.0 : -PI / 2.0);
    const double qz = std::sin(yaw / 2.0);
    const double qw = std::cos(yaw / 2.0);

    // 4. 填充里程计消息
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;

    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = qz;
    msg.pose.pose.orientation.w = qw;

    msg.twist.twist.linear.x = std::abs(omega_) * radius_;
    msg.twist.twist.angular.z = omega_;

    // 5. 广播 TF: odom -> base_link，与里程计位姿保持一致
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg.header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(tf);

    publisher_->publish(msg);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::chrono::steady_clock::time_point last_update_;
  double radius_ = 1.0;
  double omega_ = 0.5;
  double theta_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
