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

#include "Video_Saver_Node.hpp"
using std::placeholders::_1;

/// OpenCV codecs for video writing
const std::vector<std::vector<std::string>> CODECS = {
    {"h264", "H264", "avi"},
    {"xvid", "XVID", "avi"},
    {"mjpg", "MJPG", "avi"},
    {"raw", "", "avi"}};

ImageSaverNode::ImageSaverNode() : Node("number_publisher")
{
    first_message = false;
    config_found = this->declare_parameter<bool>("config_found", false);
    image_topic = this->declare_parameter<std::string>("image_topic", "image");
    output_fps = this->declare_parameter<double>("output_fps_double", 30.0);
    codec = this->declare_parameter<std::string>("codec", "mjpg");
    output_filename = this->declare_parameter<std::string>("output_filename", "/home/maimon/Videos/video_io_video");

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

    if (!config_found)
    {
        RCLCPP_WARN(this->get_logger(), "No configuration file linked, loading default parameters");
    }
    RCLCPP_INFO(get_logger(), "Saving video to %s", output_filename.c_str());

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 50, std::bind(&ImageSaverNode::topic_callback, this, _1));
}

void ImageSaverNode::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!first_message)
    {
        cv::Size S = cv::Size(msg->width, msg->height);
        // RCLCPP_INFO(get_logger(), "encoding: %s", msg->encoding.c_str());
        bool isColor;
        if (msg->encoding == "mono8")
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