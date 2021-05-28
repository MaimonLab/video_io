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
#include "video_io/color_encoding.h"

ImagePublisherNode::ImagePublisherNode() : Node("number_publisher")
{

  loop_play = this->declare_parameter<bool>("loop_play", false);
  filename = this->declare_parameter<std::string>("filename", "/home/maimon/eternarig_ws/src/video_io/videos/fictrac_bee.mp4");
  publish_as_color = this->declare_parameter<bool>("publish_as_color", true);
  start_frame = this->declare_parameter<int>("start_frame", 0);
  downsample_ratio = this->declare_parameter<double>("downsample_ratio", 1.0);

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

  // QoS settigns best effort for volatile data
  image_topic = this->declare_parameter<std::string>("image_topic", "image");
  rclcpp::QoS video_qos(1);
  video_qos.reliable();
  image_publisher = this->create_publisher<sensor_msgs::msg::Image>(image_topic, video_qos);

  dt_ms = (int)(1000.0 / publish_frequency);
  image_timer = this->create_wall_timer(std::chrono::milliseconds(dt_ms), std::bind(&ImagePublisherNode::publishImage, this));

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
  if (downsample_ratio != 1)
  {
    // Mat mat; // Input
    cv::resize(frame, resized_frame, cvSize(0, 0), downsample_ratio, downsample_ratio);
  }
  else
  {
    resized_frame = frame;
  }

  if (publish_as_color)
  {
    convert_frame_to_message(resized_frame, *img_msg);
  }
  else
  {
    cv::cvtColor(resized_frame, gray, cv::COLOR_BGR2GRAY);
    convert_frame_to_message(gray, *img_msg);
  }

  img_msg->header.frame_id = std::to_string(count);
  img_msg->header.stamp = this->get_clock()->now();

  image_publisher->publish(std::move(*img_msg));

  count++;
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