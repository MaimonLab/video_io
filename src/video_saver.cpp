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
#include "color_encoding.hpp"

#include "Video_Saver_Node.hpp"
using std::placeholders::_1;

ImageSaverNode::ImageSaverNode() : Node("number_publisher")
{
    first_message = false;

    config_found = this->declare_parameter<bool>("config_found", false);
    RCLCPP_INFO(this->get_logger(), "Config found: %i", config_found);

    this->declare_parameter("topic", "image");
    publish_topic = this->get_parameter("topic").as_string();

    this->declare_parameter("output_filename", "/home/maimon/eternarig_ws/src/video_interface/data/test_x.avi");
    output_filename = this->get_parameter("output_filename").as_string();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        publish_topic, 10, std::bind(&ImageSaverNode::topic_callback, this, _1));
}

void ImageSaverNode::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!first_message)
    {
        int fourcc = cv::VideoWriter::fourcc('X', '2', '6', '4');
        double fps = 30.0;
        cv::Size S = cv::Size(msg->width, msg->height);

        outputVideo.open(output_filename, fourcc, fps, S);
        first_message = true;
    }
    // RCLCPP_INFO(this->get_logger(), "msg properties, h: %d, w: %d", msg->height, msg->width);

    cv::Mat frame(
        msg->height, msg->width, encoding2mat_type(msg->encoding),
        const_cast<unsigned char *>(msg->data.data()), msg->step);
    outputVideo.write(frame);
}

ImageSaverNode::~ImageSaverNode()
{
    outputVideo.release();
    cv::destroyAllWindows();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}