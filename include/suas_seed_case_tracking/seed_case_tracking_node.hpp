#ifndef SUAS_SEED_CASE_TRACKING__SEED_CASE_TRACKING_NODE_HPP_
#define SUAS_SEED_CASE_TRACKING__SEED_CASE_TRACKING_NODE_HPP_

#include "suas_seed_case_tracking/camera_interface.hpp"
#include "suas_seed_case_tracking/seed_case_tracker.hpp"

#include <atomic>
#include <ctime>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>

class SeedCaseTrackingNode : public rclcpp::Node
{
public:
    enum class Mode { DataCollection, OnlineTracking };

    explicit SeedCaseTrackingNode(const rclcpp::NodeOptions& options);
    ~SeedCaseTrackingNode() override;

private:
    void eventInCallback(const std_msgs::msg::String::SharedPtr msg);
    void stopProcessing();

    // helper to convert raw image to BGR8 for saving and visualization
    const sensor_msgs::msg::Image& getBgrFrame(
        const sensor_msgs::msg::Image& frame, 
        bool& bgr_converted);

    void processingLoop();
    std::string buildFilename(const builtin_interfaces::msg::Time& stamp);

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cougarImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // mode: data collection or online tracking
    Mode mode_;

    std::unique_ptr<suas_seed_case_tracking::CameraInterface> camera_;
    std::unique_ptr<suas_seed_case_tracking::SeedCaseTracker> tracker_;

    std::thread processingThread_;
    std::atomic<bool> processing_{false};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr baslerTriggerCounterPub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr eventInSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cougarImageSub_;

    // Latest sensor data (cached)
    sensor_msgs::msg::NavSatFix::SharedPtr latestGps_;
    sensor_msgs::msg::Imu::SharedPtr latestImu_;
    sensor_msgs::msg::Image::SharedPtr latestCougarImage_;
    sensor_msgs::msg::Image bgr8Frame_; // debug image for visualization

    std::string outputFolder_;
    std::string saveImagePrefix_;
    int frameCounter_ = 0;
    int baslerTriggerCounter_ = 0;
};

#endif // SUAS_SEED_CASE_TRACKING__SEED_CASE_TRACKING_NODE_HPP_
