#ifndef SUAS_SEED_CASE_TRACKING__SEED_CASE_TRACKING_NODE_HPP_
#define SUAS_SEED_CASE_TRACKING__SEED_CASE_TRACKING_NODE_HPP_

#include "suas_seed_case_tracking/camera_interface.hpp"
#include "suas_seed_case_tracking/seed_case_tracker.hpp"

#include <atomic>
#include <ctime>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <string>
#include <queue>
#include <condition_variable>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <mavros_msgs/msg/debug_value.hpp>
#include <mavros_msgs/msg/sprouter_seed_released.hpp>
#include <mavros_msgs/msg/sprouter_status.hpp>


struct SaveRequest {
    sensor_msgs::msg::Image image;
    std::string filename;
    builtin_interfaces::msg::Time triggerTimestamp;
};

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

    void startTracking();
    std::string buildFilename(const builtin_interfaces::msg::Time& stamp);
    std::string buildCougarFilename(const builtin_interfaces::msg::Time& stamp);

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cougarImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void sprouterStatusCallback(const mavros_msgs::msg::SprouterStatus::SharedPtr msg);
    void sprouterReleasedCallback(const mavros_msgs::msg::SprouterSeedReleased::SharedPtr msg);
    void sprouterDebugValueCallback(const mavros_msgs::msg::DebugValue::SharedPtr msg);

    // mode: data collection or online tracking
    Mode mode_;

    std::unique_ptr<suas_seed_case_tracking::CameraInterface> camera_;
    std::unique_ptr<suas_seed_case_tracking::SeedCaseTracker> tracker_;

    std::thread processingThread_;
    std::atomic<bool> processing_{false};

    // Async save queue
    std::queue<SaveRequest> saveQueue_;
    std::mutex saveQueueMutex_;
    std::condition_variable saveQueueCv_;
    std::thread saveThread_;
    std::atomic<bool> saving_{false};

    // Time-windowed buffering
    double saveWindowDuration_ = 1.0;
    std::chrono::steady_clock::time_point triggerReceivedTime_;

    void saveWorker();
    void flushSaveQueue(const builtin_interfaces::msg::Time& newTriggerTimestamp);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr baslerTriggerCounterPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr eventOutPub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr eventInSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cougarImageSub_;
    rclcpp::Subscription<mavros_msgs::msg::SprouterStatus>::SharedPtr sprouterStatusSub_;
    rclcpp::Subscription<mavros_msgs::msg::SprouterSeedReleased>::SharedPtr sprouterReleasedSub_;
    rclcpp::Subscription<mavros_msgs::msg::DebugValue>::SharedPtr sprouterDebugValueSub_;

    // Latest sensor data (cached)
    sensor_msgs::msg::NavSatFix::SharedPtr latestGps_;
    sensor_msgs::msg::Imu::SharedPtr latestImu_;
    sensor_msgs::msg::Image::SharedPtr latestCougarImage_;
    sensor_msgs::msg::Image bgr8Frame_; // debug image for visualization

    std::string outputFolder_;
    std::string saveImagePrefix_;
    std::string cougarImagePrefix_;
    int frameCounter_ = 0;
    int baslerTriggerCounter_ = 0;
    bool eventTrigger_ = false;
    bool sprouterTriggerReceived_ = false;
    bool cougarSaved_ = false;
    builtin_interfaces::msg::Time sprouterTriggerTimestamp_;
    std::mutex sprouterTriggerMutex_;

    // debug params
    bool debugMode_ = false;
};

#endif // SUAS_SEED_CASE_TRACKING__SEED_CASE_TRACKING_NODE_HPP_
