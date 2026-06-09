#include "suas_seed_case_tracking/seed_case_tracking_node.hpp"
#include "suas_seed_case_tracking/basler_ace_ros2_wrapper.hpp"

SeedCaseTrackingNode::SeedCaseTrackingNode(const rclcpp::NodeOptions& options)
    : Node("seed_case_tracking_node", options)
{
    this->declare_parameter<std::string>("mode", "data_collection");
    std::string mode_str = this->get_parameter("mode").as_string();
    if (mode_str == "online_tracking") {
        mode_ = Mode::OnlineTracking;
    } else {
        mode_ = Mode::DataCollection;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Operating mode: " << mode_str);

    this->declare_parameter<std::string>("camera_type", "basler");
    std::string camera_type = this->get_parameter("camera_type").as_string();

    if (camera_type == "basler") {
        camera_ = std::make_unique<suas_seed_case_tracking::BaslerAceRos2Wrapper>(*this);
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown camera type: " << camera_type);
        throw std::runtime_error("Unknown camera type.");
    }

    if (mode_ == Mode::OnlineTracking) {
        tracker_ = std::make_unique<suas_seed_case_tracking::SeedCaseTracker>(*this);
    }

    this->declare_parameter<std::string>("output_folder", "");
    this->declare_parameter<std::string>("save_image_prefix", "frame");
    outputFolder_ = this->get_parameter("output_folder").as_string();
    saveImagePrefix_ = this->get_parameter("save_image_prefix").as_string();

    // publishers
    imagePub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 100);
    baslerTriggerCounterPub_ = this->create_publisher<std_msgs::msg::Int32>("basler_trigger_counter", 10);

    // subscribers
    eventInSub_ = this->create_subscription<std_msgs::msg::String>(
        "event_in", 10,
        std::bind(&SeedCaseTrackingNode::eventInCallback, this, std::placeholders::_1));

    std::string gps_topic, imu_topic, cougar_image_topic;
    this->declare_parameter<std::string>("gps_topic", "/mavros/global_position/global");
    this->declare_parameter<std::string>("imu_topic", "/mavros/imu/data");
    this->declare_parameter<std::string>("cougar_image_topic", "/cougar_camera/camera_driver_node/image_raw");
    gps_topic = this->get_parameter("gps_topic").as_string();
    imu_topic = this->get_parameter("imu_topic").as_string();
    cougar_image_topic = this->get_parameter("cougar_image_topic").as_string();

    gpsSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic, 10,
        std::bind(&SeedCaseTrackingNode::gpsCallback, this, std::placeholders::_1));

    imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10,
        std::bind(&SeedCaseTrackingNode::imuCallback, this, std::placeholders::_1));

    cougarImageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        cougar_image_topic, 10,
        std::bind(&SeedCaseTrackingNode::cougarImageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SeedCaseTrackingNode initialized. Waiting for events...");
}

SeedCaseTrackingNode::~SeedCaseTrackingNode()
{
    stopProcessing();
}

void SeedCaseTrackingNode::eventInCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received event: " << msg->data);

    if (msg->data == "e_start" && !processing_.load())
    {
        camera_->startGrab();
        processing_ = true;
        processingThread_ = std::thread(&SeedCaseTrackingNode::processingLoop, this);
        RCLCPP_INFO(this->get_logger(), "Processing started.");
    }
    else if (msg->data == "e_stop")
    {
        stopProcessing();
        RCLCPP_INFO(this->get_logger(), "Processing stopped.");
    }
}

void SeedCaseTrackingNode::stopProcessing()
{
    processing_ = false;
    if (processingThread_.joinable()) {
        processingThread_.join();
    }
    camera_->stopGrab();
}

const sensor_msgs::msg::Image& SeedCaseTrackingNode::getBgrFrame(
    const sensor_msgs::msg::Image& frame, 
    bool& bgr_converted)
    {
        if (!bgr_converted) {
            try {
                // Ensure the preallocated member has the correct size
                if (bgr8Frame_.data.size() != frame.width * frame.height * 3) {
                    bgr8Frame_.height = frame.height;
                    bgr8Frame_.width = frame.width;
                    bgr8Frame_.encoding = sensor_msgs::image_encodings::BGR8;
                    bgr8Frame_.is_bigendian = frame.is_bigendian;
                    bgr8Frame_.step = frame.width * 3;
                    bgr8Frame_.data.resize(bgr8Frame_.step * bgr8Frame_.height);
                }
                bgr8Frame_.header = frame.header;

                // Use cv_bridge to do the demosaicing (allocates a temporary cv::Mat)
                auto cv_img = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
                
                // Copy directly into our preallocated ROS message buffer, avoiding toImageMsg() allocation!
                std::memcpy(bgr8Frame_.data.data(), cv_img->image.data, bgr8Frame_.data.size());
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_WARN_STREAM(this->get_logger(), "cv_bridge conversion failed: " << e.what());
                return frame; // fallback to raw
            }
            bgr_converted = true;
        }
        return bgr8Frame_;
    }

void SeedCaseTrackingNode::processingLoop()
{
    sensor_msgs::msg::Image frame;
    int fpsCount = 0;
    auto lastFpsTime = std::chrono::steady_clock::now();

    while (processing_.load() && rclcpp::ok())
    {
        if (!camera_->tryGrabFrame(frame, 5000)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "tryGrabFrame timeout, retrying...");
            continue;
        }

        bool bgr_converted = false;
        baslerTriggerCounter_++;

        if (mode_ == Mode::DataCollection) {
            std::string filename = buildFilename(frame.header.stamp);
            if (imagePub_->get_subscription_count() > 0) {
                // If we have subscribers, we will demosaic anyway, so let's pass the BGR image to saveImage.
                camera_->saveImage(getBgrFrame(frame, bgr_converted), filename);
            } else {
                // No subscribers, pass raw frame to saveImage so it can demosaic directly via cv_bridge.
                camera_->saveImage(frame, filename);
            }
        } else if (mode_ == Mode::OnlineTracking) {
            // ToDo: run tracking here
            tracker_->processImage(getBgrFrame(frame, bgr_converted));
        }

        if (imagePub_->get_subscription_count() > 0) {
            imagePub_->publish(getBgrFrame(frame, bgr_converted));
        }

        // if (baslerTriggerCounterPub_->get_subscription_count() > 0) {
        //     std_msgs::msg::Int32 counter_msg;
        //     counter_msg.data = baslerTriggerCounter_;
        //     baslerTriggerCounterPub_->publish(counter_msg);
        // }

        // FPS logging: (todo: run this on debug mode only)
        fpsCount++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastFpsTime).count();
        if (elapsed >= 1.0) {
            double fps = static_cast<double>(fpsCount) / elapsed;
            RCLCPP_INFO(this->get_logger(), "Processing FPS: %.2f", fps);
            fpsCount = 0;
            lastFpsTime = now;
        }
    }
}

std::string SeedCaseTrackingNode::buildFilename(const builtin_interfaces::msg::Time& stamp)
{
    std::ostringstream ss;

    time_t sec = stamp.sec;
    struct tm tm_time;
    localtime_r(&sec, &tm_time);

    ss << outputFolder_ << "/" << saveImagePrefix_ << "_"
       << frameCounter_++ << "_"
       << std::put_time(&tm_time, "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(9) << stamp.nanosec
       << ".png";
    return ss.str();
}

void SeedCaseTrackingNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latestGps_ = msg;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "GPS: lat=%.6f lon=%.6f alt=%.2f", msg->latitude, msg->longitude, msg->altitude);
}

void SeedCaseTrackingNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    latestImu_ = msg;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "IMU: ax=%.2f ay=%.2f az=%.2f",
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

void SeedCaseTrackingNode::cougarImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    latestCougarImage_ = msg;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Cougar image: %dx%d %s", msg->width, msg->height, msg->encoding.c_str());
}

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);

    auto node = std::make_shared<SeedCaseTrackingNode>(options);

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
