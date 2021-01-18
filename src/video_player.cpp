#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "Video_Publisher_Node.hpp"

#include "color_encoding.hpp"

ImagePublisherNode::ImagePublisherNode() : Node("number_publisher")
{

  img_msg = std::make_shared<sensor_msgs::msg::Image>();
  img_msg->is_bigendian = false;

  this->declare_parameter("config_found", false);
  config_found = this->get_parameter("config_found").as_bool();

  this->declare_parameter("loop_play", false);
  loop_play = this->get_parameter("loop_play").as_bool();

  this->declare_parameter("publish_frequency", 20.0);
  publish_frequency = (double)this->get_parameter("publish_frequency").as_double();

  this->declare_parameter("topic", "image");
  publish_topic = this->get_parameter("topic").as_string();

  this->declare_parameter("filename", "/home/maimon/eternarig_ws/src/video_interface/videos/fictrac_bee.mp4");
  filename = this->get_parameter("filename").as_string();

  // check if the file can be opened, otherwise throw error
  if (FILE *tmpfile = fopen(filename.c_str(), "r"))
  {
    fclose(tmpfile);
    cap.open(filename);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Video file %s does not exist", filename.c_str());
    throw std::runtime_error("Video file does not exists");
  }

  total_n_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
  // get video parameters

  image_publisher = this->create_publisher<sensor_msgs::msg::Image>(publish_topic, 10);

  dt_ms = (int)(1000.0 / publish_frequency);
  image_timer = this->create_wall_timer(std::chrono::milliseconds(dt_ms), std::bind(&ImagePublisherNode::publishImage, this));

  RCLCPP_INFO(this->get_logger(), "Config found: %i", config_found);
  RCLCPP_INFO(this->get_logger(), "Image publisher has been started");
}

void ImagePublisherNode::publishImage()
{

  if (loop_play && (cap.get(cv::CAP_PROP_POS_FRAMES) == total_n_frames))
  {
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
  }
  else if (cap.get(cv::CAP_PROP_POS_FRAMES) == total_n_frames)
  {
    rclcpp::shutdown();
    return;
  }

  cv::Mat frame;
  cap >> frame;

  if (frame.empty())
  {
    RCLCPP_WARN(get_logger(), "could not read image");
    return;
  }

  // convert_frame_to_message(original, *img_msg);
  convert_frame_to_message(frame, *img_msg);
  image_publisher->publish(std::move(*img_msg));
}

void ImagePublisherNode::convert_frame_to_message(
    const cv::Mat &frame, sensor_msgs::msg::Image &msg)
{
  // copy cv information into ros message
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}