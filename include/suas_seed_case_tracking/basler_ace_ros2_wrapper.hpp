#pragma once

#include "suas_seed_case_tracking/camera_interface.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_integer_value.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "suas_seed_case_tracking/basler_ace.h"

namespace suas_seed_case_tracking {

/**
 * @brief ROS2-aware wrapper around BaslerCam.
 *
 * Declares camera-specific ROS2 parameters on the supplied node,
 * manages the Pylon lifecycle, and provides poll-based frame retrieval
 * via tryGrabFrame().
 */
class BaslerAceRos2Wrapper : public CameraInterface
{
public:
    /**
     * @brief Construct the wrapper.
     *
     * Declares all camera parameters on @p node, initialises Pylon,
     * opens the camera and applies the configuration.
     *
     * @throws std::runtime_error if the camera cannot be opened.
     */
    explicit BaslerAceRos2Wrapper(rclcpp::Node& node);

    ~BaslerAceRos2Wrapper();

    // Non-copyable / non-movable
    BaslerAceRos2Wrapper(const BaslerAceRos2Wrapper&) = delete;
    BaslerAceRos2Wrapper& operator=(const BaslerAceRos2Wrapper&) = delete;

    /** @brief Ensure the camera is in grabbing state. */
    void startGrab() override;

    /** @brief Stop the camera grab session. */
    void stopGrab() override;

    /**
     * @brief Try to retrieve one frame from the camera.
     *
     * Blocks for up to @p timeoutMs milliseconds waiting for a hardware
     * trigger or the next frame.
     */
    bool tryGrabFrame(sensor_msgs::msg::Image& out, int timeoutMs = 5000) override;

    /**
     * @brief Save an image to disk with Basler-specific post-processing.
     *
     * Uses Pylon's CImageFormatConverter for debayering before writing.
     */
    void saveImage(const sensor_msgs::msg::Image& image,
                   const std::string& filename) override;

    /** @brief Whether the camera grab session is active. */
    bool isRunning() const override { return running_.load(); }

    /** @brief Whether the camera is actively grabbing frames. */
    bool isGrabbing() const override;

private:
    void connectCamera();
    void declareParameters(rclcpp::Node& node);
    void applyCameraConfig();

    std::string setTriggerSource(int source);
    std::string setTriggerMode(bool mode);
    std::string setTriggerSelector(int selector);
    std::string setGrabbingStrategy(int strategy);
    std::string setUserSetSelector(int selector);
    bool isReady() const;

    void setTriggerSourceCb(const std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Request> req,
                            std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Response> res);
    void setTriggerModeCb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void setTriggerSelectorCb(const std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Request> req,
                              std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Response> res);
    void setGrabbingStrategyCb(const std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Request> req,
                               std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Response> res);
    void setUserSetSelectorCb(const std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Request> req,
                              std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Response> res);

    static std::string pixelFormatToRosEncoding(const std::string& pf);
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    std::unique_ptr<BaslerCam> camera_;
    sensor_msgs::msg::Image msg_;   // pre-allocated raw image

    std::atomic<bool> running_{false};
    std::recursive_mutex grab_mutex_;

    rclcpp::Service<pylon_ros2_camera_interfaces::srv::SetIntegerValue>::SharedPtr set_trigger_source_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_trigger_mode_srv_;
    rclcpp::Service<pylon_ros2_camera_interfaces::srv::SetIntegerValue>::SharedPtr set_trigger_selector_srv_;
    rclcpp::Service<pylon_ros2_camera_interfaces::srv::SetIntegerValue>::SharedPtr set_grabbing_strategy_srv_;
    rclcpp::Service<pylon_ros2_camera_interfaces::srv::SetIntegerValue>::SharedPtr select_user_set_srv_;

    // camera params
    std::string serial_number_;
    double gain_;
    double exposure_time_;
    int image_width_;
    int image_height_;
    std::string pixel_format_;
    std::string calibration_file_;
    int capture_duration_ms_;
    int single_capture_duration_ms_;
    int trigger_source_;
    bool trigger_mode_;
    int trigger_selector_;
    int grabbing_strategy_;
    // int batch_size_;
    int user_set_selector_;
    int black_level_;
    float gamma_;
    int acquisition_mode_;
    int acquisition_burst_frame_count_;
    bool acquisition_frame_rate_enabled_;
    int acquisition_frame_rate_;
    int shutter_mode_;
    bool pgi_mode_;
    int demosaicing_mode_;
    float noise_reduction_;
    bool debug_;
    int hw_frame_counter_ = 0;
    int hw_timestamp_ = 0;
    int frame_count_ = 0;
    std::atomic<int> savedFrameCount_{0};
};

}  // namespace suas_seed_case_tracking
