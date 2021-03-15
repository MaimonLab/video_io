#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "fic_trac/msg/latency.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "Video_Publisher_Node.hpp"
#include "video_io/color_encoding.h"

ImagePublisherNode::ImagePublisherNode() : Node("number_publisher")
{

  config_found = this->declare_parameter<bool>("config_found", false);
  loop_play = this->declare_parameter<bool>("loop_play", false);
  image_topic = this->declare_parameter<std::string>("image_topic", "image");
  filename = this->declare_parameter<std::string>("filename", "/home/maimon/eternarig_ws/src/video_io/videos/fictrac_bee.mp4");
  publish_as_color = this->declare_parameter<bool>("publish_as_color", true);
  start_frame = this->declare_parameter<int>("start_frame", 0);

  publish_latency = this->declare_parameter<bool>("publish_latency", true);
  latency_topic = this->declare_parameter<std::string>("latency_topic", "/video_player/rigX/latency");

  count = 0;

  img_msg = std::make_shared<sensor_msgs::msg::Image>();
  img_msg->is_bigendian = false;

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
  int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  double fps = cap.get(cv::CAP_PROP_FPS);
  publish_frequency = this->declare_parameter<double>("publish_frequency_double", fps);
  RCLCPP_INFO(get_logger(), "movie format: h: %d, w: %d, fps %.3f, total frames: %d", height, width, fps, total_n_frames);

  // set start frame
  cap.set(cv::CAP_PROP_POS_FRAMES, start_frame);

  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability; // want this to be reliable
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;             // want this to be keep all
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy_, depth_));
  qos.reliability(reliability_policy_);

  image_publisher = this->create_publisher<sensor_msgs::msg::Image>(image_topic, qos);
  dt_ms = (int)(1000.0 / publish_frequency);
  image_timer = this->create_wall_timer(std::chrono::milliseconds(dt_ms), std::bind(&ImagePublisherNode::publishImage, this));

  if (publish_latency)
  {
    latency_publisher = this->create_publisher<fic_trac::msg::Latency>(latency_topic, qos);
  }

  if (!config_found)
  {
    RCLCPP_WARN(this->get_logger(), "No configuration file linked, loading default parameters");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Configuration file found");
  }
  RCLCPP_INFO(get_logger(), "Playing video from %s", filename.c_str());
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

  cap.read(frame);
  if (frame.empty())
  {
    RCLCPP_WARN(get_logger(), "could not read image");
    return;
  }

  if (publish_as_color)
  {
    convert_frame_to_message(frame, *img_msg);
  }
  else
  {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    convert_frame_to_message(gray, *img_msg);
  }
  img_msg->header.frame_id = std::to_string(count);
  img_msg->header.stamp = this->get_clock()->now();

  image_publisher->publish(std::move(*img_msg));

  // double vtime = cap.get(cv::CAP_PROP_POS_MSEC);
  // RCLCPP_INFO(get_logger(), "Play count: %d", count);
  img_msg->header.frame_id = std::to_string(count);
  // img_msg->header.stamp.sec = std::floor(vtime / 1000.0);
  // img_msg->header.stamp.nanosec = std::floor((vtime / 1000.0 - std::trunc(vtime / 1000.0)) * 1000000.0);
  count++;

  if (publish_latency)
  {
    fic_trac::msg::Latency latency_msg;
    latency_msg.header = img_msg->header;
    int64_t image_timestamp = img_msg->header.stamp.sec * 1e9 + img_msg->header.stamp.nanosec;
    int64_t current_timestamp = (int64_t)get_clock()->now().nanoseconds();
    latency_msg.latency_ms = (float)(current_timestamp - image_timestamp) / 1e6;
    this->latency_publisher->publish(latency_msg);
  }
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