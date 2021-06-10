
#ifndef _VIDEO_PUBLISHER_NODE_H_
#define _VIDEO_PUBLISHER_NODE_H_

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode();
    ~ImageSaverNode();

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void burst_callback(const video_io::msg::BurstRecordCommand::SharedPtr msg);

    bool first_message;
    int fourcc;
    int record_every_nth_frame;
    int skip_counter;
    double output_fps;
    int64_t time_at_start_burst;
    int64_t time_at_end_burst;
    std::string file_extension;
    std::string image_topic;
    std::string output_filename;
    std::string output_csv_filename;
    std::string codec;
    std::string experiment_folder;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<video_io::msg::BurstRecordCommand>::SharedPtr burst_subscription;
    cv::VideoWriter outputVideo;
    std::ofstream csv_file;
};
#endif // _VIDEO_PUBLISHER_NODE_H
