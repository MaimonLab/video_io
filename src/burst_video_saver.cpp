#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <exception>
#include <vector>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "video_io/color_encoding.h"
#include <iostream>
#include <fstream>
#include "video_io/msg/burst_record_command.hpp"
#include "burst_video_saver.hpp"

using std::placeholders::_1;

/// OpenCV codecs for video writing
const std::vector<std::vector<std::string>> CODECS = {
    {"h264", "H264", "avi"},
    {"xvid", "XVID", "avi"},
    {"mjpg", "MJPG", "avi"},
    {"raw", "", "avi"}};

BurstVideoSaverNode::BurstVideoSaverNode() : Node("number_publisher")
{
    image_topic = this->declare_parameter<std::string>("image_topic", "image");
    std::string burst_record_command_topic = this->declare_parameter<std::string>("burst_record_command_topic", "video_player/burst_record_command_topic");
    output_fps = this->declare_parameter<double>("output_fps_double", 30.0);
    codec = this->declare_parameter<std::string>("codec", "mjpg");
    record_every_nth_frame = this->declare_parameter<int>("record_every_nth_frame", 0);
    output_filename = this->declare_parameter<std::string>("output_filename", "/home/maimon/Videos/video_io_video");
    verbose_logging = this->declare_parameter<bool>("verbose_logging", false);

    first_message = false;
    skip_counter = 0;
    time_at_start_burst = 0;
    time_at_end_burst = 0;

    if (verbose_logging)
    {
        RCLCPP_INFO(get_logger(), "Output csv %s", output_csv_filename.c_str());
    }

    // csv file to save timestamps from image header file
    output_csv_filename = output_filename + "_timestamps.csv";
    csv_file.open(output_csv_filename, std::ios::out);
    csv_file << "frame_id, timestamp\n";
    csv_file.close();

    for (auto codec_option : CODECS)
    {
        if (codec.compare(codec_option[0]) == 0)
        { // found the codec
            if (codec.compare("raw") != 0)
            { // codec isn't RAW
                fourcc = cv::VideoWriter::fourcc(codec_option[1][0], codec_option[1][1], codec_option[1][2], codec_option[1][3]);
            }
            else
            {
                fourcc = 0;
            }
            file_extension = codec_option[2];
        }
    }

    if (verbose_logging)
    {
        RCLCPP_INFO(get_logger(), "Saving video to %s", output_filename.c_str());
    }

    rclcpp::QoS qos_subscribe(50);
    qos_subscribe.best_effort();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, qos_subscribe, std::bind(&BurstVideoSaverNode::topic_callback, this, _1));

    rclcpp::QoS qos_burst_subscription(10);
    qos_burst_subscription.best_effort();

    burst_subscription = this->create_subscription<video_io::msg::BurstRecordCommand>(burst_record_command_topic, qos_burst_subscription, std::bind(&BurstVideoSaverNode::burst_callback, this, _1));
}

void BurstVideoSaverNode::burst_callback(const video_io::msg::BurstRecordCommand::SharedPtr msg)
{
    float record_duration = msg->record_duration_s;

    time_at_start_burst = this->now().nanoseconds();
    time_at_end_burst = time_at_start_burst + int64_t(1e9 * record_duration);

    if (verbose_logging)
    {
        RCLCPP_INFO(get_logger(), "Burst received for %f, Start time (timestamp): %zu", record_duration, time_at_start_burst);
        RCLCPP_INFO(get_logger(), "Start time, %zu", time_at_start_burst);
        RCLCPP_INFO(get_logger(), "End time  , %zu", time_at_end_burst);
    }
}

void BurstVideoSaverNode::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!first_message)
    {
        cv::Size S = cv::Size(msg->width, msg->height);
        bool isColor;
        if ((msg->encoding == "mono8") || (msg->encoding == "8UC1"))
        {

            isColor = false;
        }
        else
        {
            isColor = true;
        }

        outputVideo.open(output_filename + "." + file_extension, fourcc, output_fps, S, isColor);
        first_message = true;
    }
    skip_counter += 1;

    int64_t now_time = get_clock()->now().nanoseconds();
    bool in_burst_window = (now_time > time_at_start_burst) & (now_time < time_at_end_burst);

    if (in_burst_window)
    {
        cv::Mat frame(
            msg->height, msg->width, encoding2mat_type(msg->encoding),
            const_cast<unsigned char *>(msg->data.data()), msg->step);

        outputVideo.write(frame);

        csv_file.open(output_csv_filename, std::ios::app);
        csv_file << msg->header.frame_id;
        csv_file << ", ";
        csv_file << (int64)(msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec);
        csv_file << "\n";
        csv_file.close();
    }
    if (skip_counter >= record_every_nth_frame)
    {
        skip_counter = 0;
    }
}

BurstVideoSaverNode::~BurstVideoSaverNode()
{
    outputVideo.release();
    cv::destroyAllWindows();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BurstVideoSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}