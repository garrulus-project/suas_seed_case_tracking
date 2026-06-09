#include "suas_seed_case_tracking/seed_case_tracker.hpp"

namespace suas_seed_case_tracking {

SeedCaseTracker::SeedCaseTracker(rclcpp::Node& node)
    : logger_(node.get_logger())
{
    RCLCPP_INFO(logger_, "SeedCaseTracker initialized.");
}

void SeedCaseTracker::processImage(const sensor_msgs::msg::Image& image)
{
    // Dummy tracking implementation
    RCLCPP_DEBUG(logger_, "Processing image %dx%d for seed case tracking.", image.width, image.height);
}

} // namespace suas_seed_case_tracking
