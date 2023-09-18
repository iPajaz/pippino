#include <cstdio>
#include <chrono>
#include <memory>
#include <cmath>

#include <tf2_ros/transform_broadcaster.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define UPDATE_DT 100ms
#define PUBLISH_DT 33ms

#define PI     3.14159265
#define TWO_PI 6.28319630 

#define COUNTS_PER_METER_RIGHT 1795.0
#define COUNTS_PER_METER_LEFT 1795.0

class PippinoDriver : public rclcpp::Node
{
  private:

    void wheel_radps_left_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr position)
    {
      wheel_mps_right_ = (position->vector.x/COUNTS_PER_METER_RIGHT)/(position->vector.z/1000.0);
      wheel_mps_left_ = (position->vector.y/COUNTS_PER_METER_LEFT)/(position->vector.z/1000.0);
      // printf("speed right : %f", wheel_mps_right_);
      // printf("speed left : %f", wheel_mps_left_);
      curr_time_stamp_ = this->get_clock()->now(); //position->header.stamp;
    }

    void update_odometry() 
    {
      // Get the time delta since last update
      auto dt_ms = UPDATE_DT;
      auto dt_s = dt_ms.count() / 1000.0;
      rclcpp::Time now = this->get_clock()->now();

      // Compute distance traveled for each wheel
      // double d_right_m = wheel_mps_right_ * dt_s;
      // double d_left_m = wheel_mps_left_ * dt_s;

      double dxy = (wheel_mps_right_ + wheel_mps_left_)*dt_s / 2.0;
      double dtheta = (wheel_mps_right_ - wheel_mps_left_)*dt_s / wheelbase_m_.as_double();

      // printf("dxy=%f", dxy);

      if (dtheta > 0) dtheta *= angular_scale_positive.as_double();
      if (dtheta < 0) dtheta *= angular_scale_negative.as_double();
      if (dxy > 0) dxy *= linear_scale_positive.as_double();
      if (dxy < 0) dxy *= linear_scale_negative.as_double();

      // double theta = odom_msg_->pose.pose.orientation.z;
      double dx = dxy * std::cos(dtheta);
      double dy = dxy * std::sin(dtheta);
      // printf("dtheta = %.25f\n", dtheta);
      // printf("theta = %.25f\n", theta);

      x_pos += (std::cos(theta) * dx - std::sin(theta) * dy);
      y_pos += (std::sin(theta) * dx + std::cos(theta) * dy);

      theta += dtheta;
      if(theta >= TWO_PI) theta -= TWO_PI;
      if(theta <= -TWO_PI) theta += TWO_PI;

      // x_m += dx;
      // y_m += dy;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, theta);
      odom_msg_->pose.pose.orientation.x = q.getX();
      odom_msg_->pose.pose.orientation.y = q.getY();
      odom_msg_->pose.pose.orientation.z = q.getZ(); 
      odom_msg_->pose.pose.orientation.w = q.getW(); 
      odom_msg_->pose.pose.position.x = x_pos;
      odom_msg_->pose.pose.position.y = y_pos;
      odom_msg_->twist.twist.linear.x = (dtheta == 0)? 0: (wheel_mps_right_+wheel_mps_left_)/2;
      odom_msg_->twist.twist.angular.z = (dtheta == 0)? 0: (wheel_mps_right_ - wheel_mps_left_) / wheelbase_m_.as_double();
      odom_msg_->header.stamp = now;
      odom_msg_->header.frame_id = "odom";
      odom_msg_->child_frame_id = "base_link";


      if (wheel_mps_right_ == 0 && wheel_mps_left_ == 0){
        odom_msg_->pose.covariance[0] = 1e-9;
        odom_msg_->pose.covariance[7] = 1e-9;
        odom_msg_->pose.covariance[14] = 0.0;//1e6;
        odom_msg_->pose.covariance[21] = 0.0;//1e6;
        odom_msg_->pose.covariance[28] = 0.0;//1e6;
        odom_msg_->pose.covariance[35] = 1e-9;
        odom_msg_->twist.covariance[0] = 1e-9;
        odom_msg_->twist.covariance[7] = 1e-9;
        odom_msg_->twist.covariance[14] = 0.0;//1e6;
        odom_msg_->twist.covariance[21] = 0.0;//1e6;
        odom_msg_->twist.covariance[28] = 0.0;//1e6;
        odom_msg_->twist.covariance[35] = 1e-9;
      }
      else{
        odom_msg_->pose.covariance[0] = 1e-2;
        odom_msg_->pose.covariance[7] = 1e-5;
        odom_msg_->pose.covariance[14] = 0.0;//1e6;
        odom_msg_->pose.covariance[21] = 0.0;//1e6;
        odom_msg_->pose.covariance[28] = 0.0;//1e6;
        odom_msg_->pose.covariance[35] = 1e-3;//1e3;
        odom_msg_->twist.covariance[0] = 1e-5;
        odom_msg_->twist.covariance[7] = 1e-5;
        odom_msg_->twist.covariance[14] = 0.0;//1e6;
        odom_msg_->twist.covariance[21] = 0.0;//1e6;
        odom_msg_->twist.covariance[28] = 0.0;//1e6;
        odom_msg_->twist.covariance[35] = 1e-3;//1e3;
      }

      // // Broadcast transform:
      // geometry_msgs::msg::TransformStamped t;
      // // rclcpp::Time now = this->get_clock()->now();
      // // Read message content and assign it to
      // // corresponding tf variables4
      // t.header.stamp = now;
      // t.header.frame_id = "odom";
      // t.child_frame_id = "base_link";

      // // Pippino only exists in 2D, thus we get x and y translation
      // // coordinates from the message and set the z coordinate to 0
      // t.transform.translation.x = x_pos;
      // t.transform.translation.y = y_pos;
      // t.transform.translation.z = 0.0;

      // // For the same reason, pippino can only rotate around one axis
      // // and this why we set rotation in x and y to 0 and obtain
      // // rotation in z axis from the message
      // t.transform.rotation.x = q.getX();
      // t.transform.rotation.y = q.getY();
      // t.transform.rotation.z = q.getZ();
      // t.transform.rotation.w = q.getW();
      // tf_broadcaster_->sendTransform(t);

      // // Laser
      // geometry_msgs::msg::TransformStamped l;
      // tf2::Quaternion empty_quat;
      // empty_quat.setRPY(0.0,0.0,PI);

      // l.header.stamp = curr_time_stamp_;
      // l.header.frame_id = "/laser";
      // l.child_frame_id = "/laser_link";
      // l.transform.translation.x = -0.15;
      // l.transform.translation.y = 0.0;
      // l.transform.translation.z = 0.18;
      // l.transform.rotation.x = empty_quat.getX();
      // l.transform.rotation.y = empty_quat.getY();
      // l.transform.rotation.z = empty_quat.getZ();


      // // Send the transformation
      // tf_broadcaster_->sendTransform(l);
    }

    void publish_odometry()
    {
      odometry_pub_->publish(*odom_msg_);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr radps_left_sub_;
    rclcpp::TimerBase::SharedPtr update_odometry_timer_;
    rclcpp::TimerBase::SharedPtr publish_odometry_timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // double d_left_m;
    // double d_right_m;
    double theta;
    double wheel_mps_right_;
    double wheel_mps_left_;

    // double wheelbase_m_;

    double x_pos;
    double y_pos;

    nav_msgs::msg::Odometry::SharedPtr odom_msg_;
    rclcpp::Time curr_time_stamp_;

    rclcpp::Parameter linear_scale_positive;
    rclcpp::Parameter linear_scale_negative;
    rclcpp::Parameter angular_scale_positive;
    rclcpp::Parameter angular_scale_negative;
    rclcpp::Parameter wheelbase_m_;

  public:

    PippinoDriver() : Node("pippino_driver"), odom_msg_(std::make_shared<nav_msgs::msg::Odometry>())
    {
      this->declare_parameter<double>("linear_scale_positive");
      this->declare_parameter<double>("linear_scale_negative");
      this->declare_parameter<double>("angular_scale_positive");
      this->declare_parameter<double>("angular_scale_negative");
      this->declare_parameter<double>("wheelbase_m_");

      linear_scale_positive = this->get_parameter("linear_scale_positive");
      linear_scale_negative = this->get_parameter("linear_scale_negative");
      angular_scale_positive = this->get_parameter("angular_scale_positive");
      angular_scale_negative = this->get_parameter("angular_scale_negative");
      wheelbase_m_ = this->get_parameter("wheelbase_m_");

      theta=0;

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      radps_left_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "encoders", 10,
        std::bind(&PippinoDriver::wheel_radps_left_cb, this, _1)
      );

      odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "pippino/odom", 10);

      update_odometry_timer_ = this->create_wall_timer(
        UPDATE_DT, std::bind(&PippinoDriver::update_odometry, this));

      publish_odometry_timer_ = this->create_wall_timer(
        PUBLISH_DT, std::bind(&PippinoDriver::publish_odometry, this));
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PippinoDriver>());
  rclcpp::shutdown();
  return 0;
}