#pragma once

#include <functional>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/time.hpp>

namespace suas_seed_case_tracking {

/**
 * @brief Interface for cameras used in the seed case tracking system.
 * 
 * Generic camera interface.
 */
class CameraInterface {
public:
    virtual ~CameraInterface() = default;

    /**
     * @brief Begin the camera's internal grab session (no callback).
     */
    virtual void startGrab() = 0;

    /**
     * @brief Stop grabbing frames.
     */
    virtual void stopGrab() = 0;

    /**
     * @brief Try to retrieve one frame.
     *
     * Blocks for up to @p timeout_ms milliseconds.
     * @return true if a frame was successfully written to @p out.
     */
    virtual bool tryGrabFrame(sensor_msgs::msg::Image& out, int timeoutMs = 5000) = 0;

    /**
     * @brief Save an image to disk.
     *
     * Each camera implementation can apply its own post-processing
     * (e.g. debayering, format conversion) before writing.
     */
    virtual void saveImage(const sensor_msgs::msg::Image& image,
                           const std::string& filename) = 0;

    /**
     * @brief Check if the camera is currently running/grabbing.
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Check if the camera is actively grabbing frames.
     */
    virtual bool isGrabbing() const = 0;
};

} // namespace suas_seed_case_tracking
