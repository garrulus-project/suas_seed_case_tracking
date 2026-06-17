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
    this->declare_parameter<std::string>("cougar_image_prefix", "cougar");
    this->declare_parameter<std::string>("metadata_prefix", "metadata");
    this->declare_parameter<double>("save_window_duration", 1.0);
    outputFolder_ = this->get_parameter("output_folder").as_string();
    saveImagePrefix_ = this->get_parameter("save_image_prefix").as_string();
    cougarImagePrefix_ = this->get_parameter("cougar_image_prefix").as_string();
    metadataPrefix_ = this->get_parameter("metadata_prefix").as_string();
    saveWindowDuration_ = this->get_parameter("save_window_duration").as_double();

    // publishers
    imagePub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 100);
    baslerTriggerCounterPub_ = this->create_publisher<std_msgs::msg::Int32>("basler_trigger_counter", 10);
    eventOutPub_ = this->create_publisher<std_msgs::msg::String>("event_out", 10);

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

    std::string sprouter_status_topic, sprouter_released_topic, sprouter_debug_value_topic;
    this->declare_parameter<std::string>("sprouter_status_topic", "/mavros/sprouter/status");
    this->declare_parameter<std::string>("sprouter_released_topic", "/mavros/sprouter/seed_released");
    this->declare_parameter<std::string>("sprouter_debug_value_topic", "/mavros/debug_value/named_value_int");
    sprouter_status_topic = this->get_parameter("sprouter_status_topic").as_string();
    sprouter_released_topic = this->get_parameter("sprouter_released_topic").as_string();
    sprouter_debug_value_topic = this->get_parameter("sprouter_debug_value_topic").as_string();
    sprouterStatusSub_ = this->create_subscription<mavros_msgs::msg::SprouterStatus>(
        sprouter_status_topic, 10,
        std::bind(&SeedCaseTrackingNode::sprouterStatusCallback, this, std::placeholders::_1));
    sprouterReleasedSub_ = this->create_subscription<mavros_msgs::msg::SprouterSeedReleased>(
        sprouter_released_topic, 10,
        std::bind(&SeedCaseTrackingNode::sprouterReleasedCallback, this, std::placeholders::_1));
    sprouterDebugValueSub_ = this->create_subscription<mavros_msgs::msg::DebugValue>(
        sprouter_debug_value_topic, 10,
        std::bind(&SeedCaseTrackingNode::sprouterDebugValueCallback, this, std::placeholders::_1));
    
    this->declare_parameter<bool>("flush_queue_on_trigger", false);
    flushQueueOnTrigger_ = this->get_parameter("flush_queue_on_trigger").as_bool();

    RCLCPP_INFO(this->get_logger(), "SeedCaseTrackingNode initialized....");
    
    // this->declare_parameter<bool>("debug", false);
    debugMode_ = this->get_parameter("debug").as_bool();
    RCLCPP_INFO_STREAM(this->get_logger(), "Debug mode: " << (debugMode_ ? "enabled" : "disabled"));

    this->declare_parameter<bool>("event_trigger", false);
    eventTrigger_ = this->get_parameter("event_trigger").as_bool();
    RCLCPP_INFO_STREAM(this->get_logger(), "Event trigger: " << (eventTrigger_ ? "enabled" : "disabled"));
    if (!eventTrigger_) {
        // Start processing immediately if event trigger is disabled
        camera_->startGrab();
        processing_ = true;
        saving_ = true;
        saveThread_ = std::thread(&SeedCaseTrackingNode::saveWorker, this);
        processingThread_ = std::thread(&SeedCaseTrackingNode::startTracking, this);
        RCLCPP_INFO(this->get_logger(), "Event trigger disabled, started tracking immediately...");
    } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for 'e_start' event to start tracking...");
    }
}

SeedCaseTrackingNode::~SeedCaseTrackingNode()
{
    stopProcessing();
}

void SeedCaseTrackingNode::eventInCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received event: " << msg->data);
    if (!eventTrigger_) {
        RCLCPP_WARN(this->get_logger(), "Received 'e_start' event but event trigger is disabled. Ignoring.");
        return;
    }
    std_msgs::msg::String event_out_msg;
    if (msg->data == "e_start" && !processing_.load() && eventTrigger_)
    {
        camera_->startGrab();
        processing_ = true;
        saving_ = true;
        saveThread_ = std::thread(&SeedCaseTrackingNode::saveWorker, this);
        processingThread_ = std::thread(&SeedCaseTrackingNode::startTracking, this);
        RCLCPP_INFO(this->get_logger(), "Processing started.");
        event_out_msg.data = "e_started";
        eventOutPub_->publish(event_out_msg);
    }
    else if (msg->data == "e_stop")
    {
        stopProcessing();
        RCLCPP_INFO(this->get_logger(), "Processing stopped.");
        event_out_msg.data = "e_stopped";
        eventOutPub_->publish(event_out_msg);
    }
}

void SeedCaseTrackingNode::stopProcessing()
{
    processing_ = false;
    if (processingThread_.joinable()) {
        processingThread_.join();
    }

    // Stop the save worker
    saving_ = false;
    saveQueueCv_.notify_one();
    if (saveThread_.joinable()) {
        saveThread_.join();
    }
    // Drain remaining
    {
        std::lock_guard<std::mutex> lock(saveQueueMutex_);
        while (!saveQueue_.empty()) {
            auto req = std::move(saveQueue_.front());
            saveQueue_.pop();
            camera_->saveImage(req.image, req.filename);
        }
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

void SeedCaseTrackingNode::startTracking()
{
    sensor_msgs::msg::Image frame;
    int fpsCount = 0;
    auto trackingStartTime = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Tracking thread started. Waiting for triggered frames...");

    while (processing_.load() && rclcpp::ok())
    {
        if (!camera_->tryGrabFrame(frame, 5000)) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "tryGrabFrame timeout, retrying...");
            continue;
        }

        bool bgr_converted = false;
        // baslerTriggerCounter_++;

        if (mode_ == Mode::DataCollection) {
            std::unique_lock<std::mutex> lock(sprouterTriggerMutex_);
            if (sprouterTriggerReceived_) {
                auto elapsed = std::chrono::steady_clock::now() - triggerReceivedTime_;
                if (std::chrono::duration<double>(elapsed).count() < saveWindowDuration_) {
                    std::string filename = buildFilename(sprouterTriggerTimestamp_);
                    auto currentTriggerTs = sprouterTriggerTimestamp_;
                    lock.unlock();
                    {
                        std::lock_guard<std::mutex> qLock(saveQueueMutex_);
                        saveQueue_.push({frame, std::move(filename), currentTriggerTs});
                    }
                    saveQueueCv_.notify_one();
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Enqueued frame for saving");
                }
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
        if (debugMode_)
        {
            fpsCount++;
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - trackingStartTime).count();
            if (elapsed >= 1.0) {
                double fps = static_cast<double>(fpsCount) / elapsed;
                RCLCPP_INFO(this->get_logger(), "Processing FPS: %.2f frame counts: %d", fps, fpsCount);
                fpsCount = 0;
                trackingStartTime = now;
            }
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
       << std::put_time(&tm_time, "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(9) << stamp.nanosec << "_"
       << frameCounter_++
       << ".png";
   return ss.str();
}

std::string SeedCaseTrackingNode::buildCougarFilename(const builtin_interfaces::msg::Time& stamp)
{
    std::ostringstream ss;

    time_t sec = stamp.sec;
    struct tm tm_time;
    localtime_r(&sec, &tm_time);

    ss << outputFolder_ << "/" << cougarImagePrefix_ << "_"
       << std::put_time(&tm_time, "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(9) << stamp.nanosec
       << ".png";
    return ss.str();
}

std::string SeedCaseTrackingNode::buildMetadataFilename(const builtin_interfaces::msg::Time& stamp)
{
    std::ostringstream ss;

    time_t sec = stamp.sec;
    struct tm tm_time;
    localtime_r(&sec, &tm_time);

    ss << outputFolder_ << "/" << metadataPrefix_ << "_"
       << std::put_time(&tm_time, "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(9) << stamp.nanosec
       << ".json";
    return ss.str();
}

void SeedCaseTrackingNode::saveMetadata(const mavros_msgs::msg::SprouterSeedReleased::SharedPtr& released_msg)
{
    nlohmann::json j;

    j["sprouter_released"] = {
        {"header_stamp", {{"sec", released_msg->header.stamp.sec}, {"nanosec", released_msg->header.stamp.nanosec}}},
        {"timestamp", released_msg->timestamp},
        {"seed_count", released_msg->seed_count},
        {"distance", released_msg->distance}
    };

    if (triggerGps_) {
        std::vector<double> cov;
        for (int i = 0; i < 9; ++i) {
            cov.push_back(triggerGps_->position_covariance[i]);
        }
        j["gps"] = {
            {"header_stamp", {{"sec", triggerGps_->header.stamp.sec}, {"nanosec", triggerGps_->header.stamp.nanosec}}},
            {"latitude", triggerGps_->latitude},
            {"longitude", triggerGps_->longitude},
            {"altitude", triggerGps_->altitude},
            {"status", {{"status", triggerGps_->status.status}, {"service", triggerGps_->status.service}}},
            {"position_covariance", cov},
            {"position_covariance_type", triggerGps_->position_covariance_type}
        };
    } else {
        j["gps"] = nullptr;
    }

    if (triggerImu_) {
        j["imu"] = {
            {"header_stamp", {{"sec", triggerImu_->header.stamp.sec}, {"nanosec", triggerImu_->header.stamp.nanosec}}},
            {"orientation", {
                {"x", triggerImu_->orientation.x},
                {"y", triggerImu_->orientation.y},
                {"z", triggerImu_->orientation.z},
                {"w", triggerImu_->orientation.w}
            }},
            {"angular_velocity", {
                {"x", triggerImu_->angular_velocity.x},
                {"y", triggerImu_->angular_velocity.y},
                {"z", triggerImu_->angular_velocity.z}
            }},
            {"linear_acceleration", {
                {"x", triggerImu_->linear_acceleration.x},
                {"y", triggerImu_->linear_acceleration.y},
                {"z", triggerImu_->linear_acceleration.z}
            }}
        };
    } else {
        j["imu"] = nullptr;
    }

    std::string filename = buildMetadataFilename(sprouterTriggerTimestamp_);
    std::ofstream file(filename);
    if (file.is_open()) {
        file << j.dump(4);
        file.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Saved metadata to " << filename);
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open metadata file: " << filename);
    }
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

    std::lock_guard<std::mutex> lock(sprouterTriggerMutex_);
    if (sprouterTriggerReceived_ && !cougarSaved_) {
        std::string cougarFilename = buildCougarFilename(sprouterTriggerTimestamp_);
        camera_->saveImage(*msg, cougarFilename);
        cougarSaved_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "Cougar image saved: " << cougarFilename);
    }
}

void SeedCaseTrackingNode::sprouterStatusCallback(const mavros_msgs::msg::SprouterStatus::SharedPtr msg)
{
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Sprouter status: current state=%d", msg->current_state);
}

void SeedCaseTrackingNode::sprouterReleasedCallback(const mavros_msgs::msg::SprouterSeedReleased::SharedPtr msg)
{
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Sprouter released: seed_count=%d", msg->seed_count);
    RCLCPP_INFO_STREAM(this->get_logger(), "Sprouter released event received, seed count: " << msg->seed_count);
    std::lock_guard<std::mutex> lock(sprouterTriggerMutex_);
    if (sprouterTriggerReceived_) {
        saveMetadata(msg);
    }
}

void SeedCaseTrackingNode::sprouterDebugValueCallback(const mavros_msgs::msg::DebugValue::SharedPtr msg)
{
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Sprouter debug value: name=%s value=%d", msg->name.c_str(), msg->value_int);
    if (msg->name == "SprTrigSgl" && msg->value_int == 1) {
        {
            std::lock_guard<std::mutex> lock(sprouterTriggerMutex_);
            sprouterTriggerReceived_ = true;
            sprouterTriggerTimestamp_ = msg->header.stamp;
            triggerReceivedTime_ = std::chrono::steady_clock::now();
            cougarSaved_ = false;
            frameCounter_ = 0; // reset frame counter for filename
            triggerGps_ = latestGps_;   // snapshot GPS at trigger time
            triggerImu_ = latestImu_;   // snapshot IMU at trigger time
        }
        if (flushQueueOnTrigger_) {
            flushSaveQueue(msg->header.stamp);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(),  *this->get_clock(), 1000,
                "Flushing frames from queue...");
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Sprouter trigger received, timestamp: "
            << msg->header.stamp.sec << "." << msg->header.stamp.nanosec);
        RCLCPP_INFO_STREAM(this->get_logger(), "Remaining unsaved images in the queue: "
            << saveQueue_.size());
    }
}

void SeedCaseTrackingNode::saveWorker()
{
    while (true) {
        SaveRequest req;
        {
            std::unique_lock<std::mutex> lock(saveQueueMutex_);
            saveQueueCv_.wait(lock, [this] {
                return !saveQueue_.empty() || !saving_.load();
            });
            if (saveQueue_.empty() && !saving_.load()) {
                break;
            }
            if (saveQueue_.empty()) {
                continue;
            }
            req = std::move(saveQueue_.front());
            saveQueue_.pop();
        }
        camera_->saveImage(req.image, req.filename);
    }
}

void SeedCaseTrackingNode::flushSaveQueue(
    const builtin_interfaces::msg::Time& newTriggerTimestamp)
{
    std::lock_guard<std::mutex> lock(saveQueueMutex_);
    size_t discarded = 0;
    std::queue<SaveRequest> keep;
    while (!saveQueue_.empty()) {
        auto& front = saveQueue_.front();
        if (front.triggerTimestamp.sec == newTriggerTimestamp.sec &&
            front.triggerTimestamp.nanosec == newTriggerTimestamp.nanosec) {
            keep.push(std::move(front));
        } else {
            discarded++;
        }
        saveQueue_.pop();
    }
    saveQueue_ = std::move(keep);
    if (discarded > 0) {
        RCLCPP_WARN(this->get_logger(),
            "Flushed %zu stale frames from save queue on new trigger", discarded);
    }
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
