

#ifndef _VIDEO_PUBLISHER_NODE_H_
#define _VIDEO_PUBLISHER_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <deque>

class MultithreadVideoSaverNode : public rclcpp::Node
{
public:
    MultithreadVideoSaverNode();
    ~MultithreadVideoSaverNode();
    // ImageSaverNode();
    // ~ImageSaverNode();

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void timer_callback();

    bool first_message;
    int fourcc;
    int counter;
    double output_fps;
    std::string file_extension;
    std::string image_topic;
    std::string output_filename;
    std::string codec;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter outputVideo;

    // mutex

    size_t depth_ = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
    // queue
    std::deque<std::shared_ptr<cv::Mat>> image_pointer_queue;
    std::shared_ptr<cv::Mat> image_pointer;
    std::shared_ptr<cv::Mat> receiving_image_pointer;

    std::mutex pointer_queue_mutex;

    int publish_interval_ms;

    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame;
    // std::deque<cv::Mat> > image_pointer_queue;
    // std::shared_ptr<cv::Mat>
};
#endif // _VIDEO_PUBLISHER_NODE_H
