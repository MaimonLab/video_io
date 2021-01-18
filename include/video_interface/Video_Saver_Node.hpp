
#ifndef _VIDEO_PUBLISHER_NODE_H_
#define _VIDEO_PUBLISHER_NODE_H_

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode();
    ~ImageSaverNode();

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    bool config_found;
    bool first_message;
    std::string publish_topic;
    std::string output_filename;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter outputVideo;
};
#endif // _VIDEO_PUBLISHER_NODE_H
