
#ifndef _VIDEO_PUBLISHER_NODE_H_
#define _VIDEO_PUBLISHER_NODE_H_

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode();

private:
    void publishImage();
    void convert_frame_to_message(const cv::Mat &frame, sensor_msgs::msg::Image &img_msg);

    double publish_frequency;
    std::string publish_topic;
    bool config_found;
    bool loop_play;
    int dt_ms;
    int total_n_frames;
    std::string filename;

    cv::VideoCapture cap;

    cv::Mat original;
    rclcpp::TimerBase::SharedPtr image_timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;

    std::shared_ptr<sensor_msgs::msg::Image> img_msg;
};

#endif // _VIDEO_PUBLISHER_NODE_H
