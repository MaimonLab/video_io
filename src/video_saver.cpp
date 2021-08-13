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

#include "Video_Saver_Node.hpp"
using std::placeholders::_1;

/// OpenCV codecs for video writing
const std::vector<std::vector<std::string>> CODECS = {
    {"h264", "H264", "avi"},
    {"xvid", "XVID", "avi"},
    {"mjpg", "MJPG", "avi"},
    {"raw", "", "avi"}};

VideoSaverNode::VideoSaverNode() : Node("number_publisher")
{
    first_message = false;
    image_topic = this->declare_parameter<std::string>("image_topic", "image");
    output_fps = this->declare_parameter<double>("output_fps_double", 30.0);
    codec = this->declare_parameter<std::string>("codec", "mjpg");
    record_every_nth_frame = this->declare_parameter<int>("record_every_nth_frame", 1);
    burn_timestamp = this->declare_parameter<bool>("burn_timestamp", false);
    skip_counter = 0;
    output_filename = this->declare_parameter<std::string>("output_filename", "/home/maimon/Videos/video_io_video");
    verbose_logging = this->declare_parameter<bool>("verbose_logging", false);

    skip_counter = 0;

    // create a csv file to log image header timestamps
    output_csv_filename = output_filename + "_timestamps.csv";
    csv_file.open(output_csv_filename, std::ios::out);
    csv_file << "frame_id, timestamp\n";
    csv_file.close();

    if (verbose_logging)
    {
        RCLCPP_INFO(get_logger(), "Saving csv to %s", output_csv_filename.c_str());
    }

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
    if (burn_timestamp){
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(buffer, 80, "%m/%d/%Y %H:%M:%S",timeinfo);
        std::string time_string(buffer);
        text_size = cv::getTextSize(time_string, cv::FONT_HERSHEY_PLAIN, 
                                            font_scale, 1, &baseline);
    }

    
    rclcpp::QoS qos_subscribe(50);
    qos_subscribe.best_effort();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, qos_subscribe, std::bind(&VideoSaverNode::topic_callback, this, _1));
}

void VideoSaverNode::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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
    if (skip_counter == record_every_nth_frame)
    {
        cv::Mat frame(
            msg->height, msg->width, encoding2mat_type(msg->encoding),
            const_cast<unsigned char *>(msg->data.data()), msg->step);
        
        
        
        if (burn_timestamp){
            time(&rawtime);
            timeinfo = localtime(&rawtime);
            strftime(buffer, 80, "%m/%d/%Y %H:%M:%S",timeinfo);
            std::string time_string(buffer);
                    
            cv::rectangle(frame, cv::Point(0, frame.rows ), 
                                cv::Point(text_size.width, frame.rows - (text_size.height + 5)), 
                                cv::Scalar(0,0,0), -1);
            cv::putText(frame, time_string, cv::Point(0, frame.rows), 
                        cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255,255,255), thickness);
        }

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

VideoSaverNode::~VideoSaverNode()
{
    outputVideo.release();
    cv::destroyAllWindows();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}