#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rclcpp/parameter.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "cv_bridge/cv_bridge.h"

// for logging to msg
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"

// Pippino specific
#include "pippino_service_msg/srv/pippino_video_stream.hpp"

    using std::placeholders::_1, std::placeholders::_2;

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>


using namespace std::chrono_literals;
using namespace std;

class VideoStreamController : public rclcpp::Node
{
public:
  VideoStreamController() : Node("video_stream_controller")
  {
    this->declare_parameter("d455_fast_profile", "");
    this->declare_parameter("d455_hires_profile", "");
    this->declare_parameter("color_image_topic", "");
    this->declare_parameter("fisheye_image_topic", "");
    this->declare_parameter("fisheye_cam_calib_param_file", "");

    d455_fast_profile = this->get_parameter("d455_fast_profile").as_string();
    d455_hires_profile = this->get_parameter("d455_hires_profile").as_string();
    color_image_topic = this->get_parameter("color_image_topic").as_string();
    fisheye_image_topic = this->get_parameter("fisheye_image_topic").as_string();
    fisheye_cam_calib_param_file = this->get_parameter("fisheye_cam_calib_param_file").as_string();

    // RCLCPP_INFO(this->get_logger(), "Hello %s!", d455_fast_profile.c_str());
    // Read cv file for cam settings.
    cv::FileStorage cv_file(fisheye_cam_calib_param_file, cv::FileStorage::READ);
    cv_file["K"] >> mtx;
    cv_file["D"] >> dst;
    cv_file.release();

    cout << "\n -- Camera parameters -- " << endl;
    cout << "\n CameraMatrix = " << endl << " " << mtx << endl << endl;
    cout << " Distortion coefficients = " << endl << " " << dst << endl << endl;

    // publishers
    compressed_image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("/strctl/image_raw/compressed", 1);
    cam_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("/strctl/image_raw/camera_info", 1);

    // subscribers
    // color_subscription = this->create_subscription<sensor_msgs::msg::Image>(
    //     color_image_topic, 5, std::bind(&VideoStreamController::color_image_callback, this, _1));
    fisheye_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        fisheye_image_topic, 5, std::bind(&VideoStreamController::fisheye_image_callback, this, _1));

    // std::cout<<mtx<<std::endl;
    // control service
    video_stream_service = this->create_service<pippino_service_msg::srv::PippinoVideoStream>("pippino_video_stream", std::bind(&VideoStreamController::control_service_callback, this, _1, _2));
  }

private:
  // void color_image_callback(const sensor_msgs::msg::Image::SharedPtr image){
  //   cout<<"received color image"<<endl;
  //   cv_bridge::

  //   sensor_msgs::msg::CompressedImage img_msg;
  //   img_bri
  //   auto current_frame =

  // }
  void fisheye_image_callback(const sensor_msgs::msg::Image::SharedPtr image)
  {
    cout<<"received fisheye image"<<endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image, "mono8");
    cv::Mat output_image;
    cv::undistort(cv_ptr->image, output_image, mtx, dst);
    sensor_msgs::msg::CompressedImage::SharedPtr compressed_image_msg = cv_bridge::CvImage(image->header, "mono8", output_image).toCompressedImageMsg();
    cout<<"Format: "<<compressed_image_msg->format<<endl;
    compressed_image_publisher->publish(*compressed_image_msg);
  }
  void control_service_callback(const std::shared_ptr<pippino_service_msg::srv::PippinoVideoStream::Request> request,
                                std::shared_ptr<pippino_service_msg::srv::PippinoVideoStream::Response> response)
  {
    cout<<"received request for control"<<endl;
  }
  std::string curr_color_profile = "";

  std::string d455_fast_profile;
  std::string d455_hires_profile;
  std::string color_image_topic;
  std::string fisheye_image_topic;
  std::string fisheye_cam_calib_param_file;

  cv::Mat mtx, dst;


  // publishers
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_publisher;

  // subscribers
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr fisheye_subscription;

  // control service
  rclcpp::Service<pippino_service_msg::srv::PippinoVideoStream>::SharedPtr video_stream_service;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoStreamController>());
  rclcpp::shutdown();
  return 0;
}