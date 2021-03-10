
#ifndef _VIDEO_PUBLISHER_NODE_H_
#define _VIDEO_PUBLISHER_NODE_H_

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode();

private:
    void publishImage();
    void convert_frame_to_message(const cv::Mat &frame, sensor_msgs::msg::Image &img_msg);

    bool config_found;
    bool loop_play;
    bool publish_as_color;
    bool publish_latency;
    int dt_ms;
    int total_n_frames;
    int count;
    int start_frame;
    double publish_frequency;
    std::string filename;
    std::string publish_topic;
    std::string latency_topic_name;
    cv::VideoCapture cap;
    cv::Mat frame, gray;
    rclcpp::TimerBase::SharedPtr image_timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<fic_trac::msg::Latency>::SharedPtr latency_publisher;
    std::shared_ptr<sensor_msgs::msg::Image> img_msg;
};

#endif // _VIDEO_PUBLISHER_NODE_H