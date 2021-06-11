
#ifndef _VIDEO_PUBLISHER_NODE_H_
#define _VIDEO_PUBLISHER_NODE_H_

class VideoSaverNode : public rclcpp::Node
{
public:
    VideoSaverNode();
    ~VideoSaverNode();

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    bool first_message;
    bool verbose_logging;
    int fourcc;
    int record_every_nth_frame;
    int skip_counter;
    double output_fps;
    std::string file_extension;
    std::string image_topic;
    std::string output_filename;
    std::string output_csv_filename;
    std::string codec;
    std::string experiment_folder;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter outputVideo;

    std::ofstream csv_file;
};
#endif // _VIDEO_PUBLISHER_NODE_H
