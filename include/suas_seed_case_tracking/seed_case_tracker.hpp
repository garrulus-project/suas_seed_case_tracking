#pragma once

#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

namespace suas_seed_case_tracking {

/**
 * @brief Tracker for seed cases from camera images.
 */
class SeedCaseTracker {
public:
    SeedCaseTracker(rclcpp::Node& node);
    ~SeedCaseTracker() = default;

    /**
     * @brief Process an incoming image frame.
     */
    void processImage(const sensor_msgs::msg::Image& image);

private:
    rclcpp::Logger logger_;
};

} // namespace suas_seed_case_tracking
