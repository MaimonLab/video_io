#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "video_interface/color_encoding.h"
#include <deque>

#include <chrono>

#include "cv_bridge/cv_bridge.h"

#include "video_interface/multithread_video_saver.hpp"

const std::vector<std::vector<std::string>> CODECS = {
    {"h264", "H264", "avi"},
    {"xvid", "XVID", "avi"},
    {"mjpg", "MJPG", "avi"},
    {"raw", "", "avi"}};

using std::placeholders::_1;

MultithreadVideoSaverNode::MultithreadVideoSaverNode(const rclcpp::NodeOptions &options) : Node("video_saver")
{
    first_message = false;
    config_found = this->declare_parameter<bool>("config_found", false);
    publish_topic = this->declare_parameter<std::string>("topic", "image");
    output_filename = this->declare_parameter<std::string>("output_filename", "/home/maimon/eternarig_ws/src/video_interface/data/test_x");
    output_fps = this->declare_parameter<double>("output_fps_double", 30.0);
    codec = this->declare_parameter<std::string>("codec", "mjpg");

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
    else
    {
        RCLCPP_INFO(this->get_logger(), "Configuration file found");
    }
    RCLCPP_INFO(get_logger(), "Saving video to %s", output_filename.c_str());

    auto sub1_opt = rclcpp::SubscriptionOptions();
    auto cb_group_sub = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    sub1_opt.callback_group = cb_group_sub;

    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_default.history, rmw_qos_profile_default.reliability));
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_default.history, rmw_qos_profile_default.depth));
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_default.history, 60);
    // rclcpp:QoS(rclcpp::QoSInitialization(lkp))l
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.reliability));
    int qos = 50;

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        publish_topic, 1000, std::bind(&MultithreadVideoSaverNode::topic_callback, this, _1), sub1_opt);

    if (true)
    {
        // m_debug_publisher = create_publisher<sensor_msgs::msg::Image>(debug_topic_name, qos);
        double publish_frequency = 30;
        publish_interval_ms = (int)1000.0 / publish_frequency;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_interval_ms), std::bind(&MultithreadVideoSaverNode::timer_callback, this));
    }
}

void MultithreadVideoSaverNode::timer_callback()
{
    bool take_action = false;
    {
        std::lock_guard<std::mutex> l(pointer_queue_mutex);
        if (image_pointer_queue.size() != 0)
        {

            // get pointer from queue
            receiving_image_pointer = image_pointer_queue.front();
            take_action = true;
            image_pointer_queue.pop_front();
        }
    }

    if (take_action)
    {
        RCLCPP_INFO(get_logger(), "queue length: %d", image_pointer_queue.size());
        outputVideo.write(*receiving_image_pointer);
    }
}

void MultithreadVideoSaverNode::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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

    // cv::Mat frame(
    //     msg->height, msg->width, encoding2mat_type(msg->encoding),
    //     const_cast<unsigned char *>(msg->data.data()), msg->step);
    cv::Mat source_frame(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()), msg->step);
    counter++;
    RCLCPP_INFO(get_logger(), "ID: %s, counter %d", msg->header.frame_id.c_str(), counter);

    cv::Mat copied_frame = source_frame.clone();
    auto image_pointer = std::make_shared<cv::Mat>(copied_frame);

    {
        std::lock_guard<std::mutex> l(pointer_queue_mutex);
        image_pointer_queue.push_back(image_pointer);
    }
}

MultithreadVideoSaverNode::~MultithreadVideoSaverNode()
{
    outputVideo.release();
    cv::destroyAllWindows();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<MultithreadVideoSaverNode>(rclcpp::NodeOptions());

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}