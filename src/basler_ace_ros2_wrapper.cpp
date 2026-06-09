#include "suas_seed_case_tracking/basler_ace_ros2_wrapper.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sys/stat.h>
#include <thread>

using namespace Basler_UniversalCameraParams;

namespace suas_seed_case_tracking {


BaslerAceRos2Wrapper::BaslerAceRos2Wrapper(rclcpp::Node& node)
    : logger_(node.get_logger()),
      clock_(node.get_clock())
{
    declareParameters(node);

    Pylon::PylonInitialize();

    set_trigger_source_srv_ = node.create_service<pylon_ros2_camera_interfaces::srv::SetIntegerValue>(
        "~/set_trigger_source", std::bind(&BaslerAceRos2Wrapper::setTriggerSourceCb, this, std::placeholders::_1, std::placeholders::_2));
    
    set_trigger_mode_srv_ = node.create_service<std_srvs::srv::SetBool>(
        "~/set_trigger_mode", std::bind(&BaslerAceRos2Wrapper::setTriggerModeCb, this, std::placeholders::_1, std::placeholders::_2));
    
    set_trigger_selector_srv_ = node.create_service<pylon_ros2_camera_interfaces::srv::SetIntegerValue>(
        "~/set_trigger_selector", std::bind(&BaslerAceRos2Wrapper::setTriggerSelectorCb, this, std::placeholders::_1, std::placeholders::_2));
    
    set_grabbing_strategy_srv_ = node.create_service<pylon_ros2_camera_interfaces::srv::SetIntegerValue>(
        "~/set_grabbing_strategy", std::bind(&BaslerAceRos2Wrapper::setGrabbingStrategyCb, this, std::placeholders::_1, std::placeholders::_2));
    
    select_user_set_srv_ = node.create_service<pylon_ros2_camera_interfaces::srv::SetIntegerValue>(
        "~/select_user_set", std::bind(&BaslerAceRos2Wrapper::setUserSetSelectorCb, this, std::placeholders::_1, std::placeholders::_2));


    connectCamera();
}

void BaslerAceRos2Wrapper::connectCamera()
{
    bool is_connected = false;
    bool is_first_try = true;

    while (!is_connected && rclcpp::ok()) {
        try {
            camera_ = std::make_unique<BaslerCam>(serial_number_);
            applyCameraConfig();

            // Determine the actual pixel format and pixel depth from the camera.
            // The requested pixel_format_ may differ from what the camera
            // negotiated (e.g. the camera may only support BayerRG8 even if
            // "RGB8" was requested).
            std::string actual_pixel_format;
            try {
                actual_pixel_format = std::string(
                    camera_->getCamera().PixelFormat.ToString().c_str());
                RCLCPP_INFO_STREAM(logger_,
                    "Actual camera pixel format: " << actual_pixel_format);
            } catch (...) {
                actual_pixel_format = pixel_format_;
            }

            // PixelSize includes all channels (e.g. BayerRG8 → 8 bits = 1 byte,
            // RGB8 → 24 bits = 3 bytes).
            int pixel_depth = 1;
            try {
                pixel_depth = camera_->getCamera().PixelSize.GetIntValue() / 8;
                RCLCPP_INFO_STREAM(logger_,
                    "Camera pixel depth (bytes): " << pixel_depth);
            } catch (const Pylon::GenericException& e) {
                RCLCPP_WARN_STREAM(logger_,
                    "Could not read PixelSize: " << e.GetDescription());
            }

            // Initialise pre-allocated image message
            msg_.header.frame_id = "basler_camera";
            msg_.width  = image_width_;
            msg_.height = image_height_;
            msg_.encoding = pixelFormatToRosEncoding(actual_pixel_format);
            msg_.step = msg_.width * pixel_depth;
            msg_.data.resize(msg_.step * msg_.height);

            camera_->setupConverter();
            RCLCPP_INFO(logger_, "Camera opened and ready...");
            is_connected = true;
        } catch (const Pylon::GenericException& e) {
            if (is_first_try) {
                RCLCPP_WARN_STREAM(logger_, "Failed to connect camera device with device user id: " << serial_number_ << ". Wait and retry to connect until the specified camera is available...");
                is_first_try = false;
            } else {
                RCLCPP_WARN_STREAM(logger_, "Failed to connect camera device with device user id: " << serial_number_ << ". Trying again in a bit...");
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        } catch (const std::exception& e) {
            if (is_first_try) {
                RCLCPP_WARN_STREAM(logger_, "Failed to connect camera device with device user id: " << serial_number_ << ". Wait and retry to connect until the specified camera is available...");
                is_first_try = false;
            } else {
                RCLCPP_WARN_STREAM(logger_, "Failed to connect camera device with device user id: " << serial_number_ << ". Trying again in a bit...");
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
}

BaslerAceRos2Wrapper::~BaslerAceRos2Wrapper()
{
    stopGrab();
    if (camera_) {
        camera_->close();
    }
    Pylon::PylonTerminate();
}

void BaslerAceRos2Wrapper::startGrab()
{
    std::lock_guard<std::recursive_mutex> lock(grab_mutex_);
    if (running_.load()) return;
    
    // reset stats
    frame_count_ = 0;
    savedFrameCount_.store(0);

    // Provide the selected grabbing strategy when restarting
    camera_->restartGrabbing(grabbing_strategy_);
    running_ = true;
    RCLCPP_INFO(logger_, "Camera grab session started.");
}

void BaslerAceRos2Wrapper::stopGrab()
{
    std::lock_guard<std::recursive_mutex> lock(grab_mutex_);
    if (!running_.load()) return;

    running_ = false;
    camera_->stopGrabbingSafe();
    RCLCPP_INFO(logger_, "Camera grab session stopped.");
}

bool BaslerAceRos2Wrapper::tryGrabFrame(sensor_msgs::msg::Image& out, int timeoutMs)
{
    if (!running_.load() || !camera_) {
        return false;
    }

    Pylon::CGrabResultPtr grabResultPtr;

    try {
        camera_->getCamera().RetrieveResult(
            timeoutMs, grabResultPtr, Pylon::TimeoutHandling_Return);
    } catch (const Pylon::GenericException& e) {
        RCLCPP_WARN_STREAM(logger_, "RetrieveResult error: " << e.GetDescription());
        
        if (camera_->getCamera().IsCameraDeviceRemoved()) {
            RCLCPP_ERROR(logger_, "Camera device was removed! Attempting to reconnect...");
            camera_.reset(); // Destroy old instance to release handles
            connectCamera(); // Enters the wait-and-retry loop until reconnected
            startGrab();     // Resume the grab session automatically
        }
        return false;
    }

    if (!grabResultPtr || !grabResultPtr->GrabSucceeded()) {
        return false;
    }

    const uint8_t* rawBuf = static_cast<const uint8_t*>(grabResultPtr->GetBuffer());
    uint32_t h = grabResultPtr->GetHeight();

    // Copy raw data into pre-allocated message
    std::memcpy(msg_.data.data(), rawBuf, msg_.step * h);
    msg_.header.stamp = clock_->now();

    // if (grabResultPtr->ChunkFramecounter.IsReadable())
    // {
    //     std::cout << "FrameCounter (Result): " << grabResultPtr->ChunkFramecounter.GetValue() << std::endl;
    // }

    out = msg_;
    frame_count_++;
    return true;
}

void BaslerAceRos2Wrapper::saveImage(const sensor_msgs::msg::Image& image,
                                     const std::string& filename)
{
    try {
        cv::Mat cv_img;
        if (image.encoding == sensor_msgs::image_encodings::BGR8) {
            // Zero-copy wrap if already BGR8
            cv_img = cv::Mat(image.height, image.width, CV_8UC3, 
                             const_cast<uint8_t*>(image.data.data()), image.step);
        } else {
            // Demosaic and copy
            cv_img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
        }

        cv::imwrite(filename, cv_img);
        savedFrameCount_.fetch_add(1);

        RCLCPP_DEBUG_STREAM(logger_, "Saved image: " << filename);
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_STREAM(logger_, "cv_bridge error saving image: " << e.what());
    }
    catch (const std::exception& e) {
        RCLCPP_WARN_STREAM(logger_, "Error saving image: " << e.what());
    }

}

bool BaslerAceRos2Wrapper::isGrabbing() const
{
    return camera_ && camera_->isGrabbing();
}

void BaslerAceRos2Wrapper::declareParameters(rclcpp::Node& node)
{
    serial_number_              = node.declare_parameter<std::string>("serial_number", "40316399");
    gain_                       = node.declare_parameter<double>("gain", 0.0);
    exposure_time_              = node.declare_parameter<double>("exposure_time", 0.0);
    image_width_                = node.declare_parameter<int>("image_width", 720);
    image_height_               = node.declare_parameter<int>("image_height", 480);
    pixel_format_               = node.declare_parameter<std::string>("pixel_format", "BayerRG8");
    calibration_file_           = node.declare_parameter<std::string>("calibration_file", "");
    capture_duration_ms_        = node.declare_parameter<int>("capture_duration_ms", 60000);
    single_capture_duration_ms_ = node.declare_parameter<int>("single_capture_duration_ms", 10000);
    trigger_source_             = node.declare_parameter<int>("trigger_source", 1);
    trigger_mode_               = node.declare_parameter<bool>("trigger_mode", true);
    trigger_selector_           = node.declare_parameter<int>("trigger_selector", 1);
    grabbing_strategy_          = node.declare_parameter<int>("grabbing_strategy", 0);
    // batch_size_                 = node.declare_parameter<int>("batch_size", 200);
    user_set_selector_          = node.declare_parameter<int>("user_set_selector", 0);
    black_level_                = node.declare_parameter<int>("black_level", 0);
    gamma_                      = node.declare_parameter<float>("gamma", 1.0f);
    acquisition_mode_           = node.declare_parameter<int>("acquisition_mode", 0);
    acquisition_burst_frame_count_ = node.declare_parameter<int>("acquisition_burst_frame_count", 200);
    acquisition_frame_rate_enabled_ = node.declare_parameter<bool>("acquisition_frame_rate_enabled", false);
    acquisition_frame_rate_     = node.declare_parameter<int>("acquisition_frame_rate", 200);
    shutter_mode_               = node.declare_parameter<int>("shutter_mode", 0);
    pgi_mode_                   = node.declare_parameter<bool>("pgi_mode", false);
    demosaicing_mode_           = node.declare_parameter<int>("demosaicing_mode", 0);
    noise_reduction_            = node.declare_parameter<float>("noise_reduction", 0.0f);
    debug_                      = node.declare_parameter<bool>("debug", false);

    RCLCPP_INFO_STREAM(logger_, "Serial: " << serial_number_);
    RCLCPP_INFO_STREAM(logger_, "Gain: " << gain_);
    RCLCPP_INFO_STREAM(logger_, "Exposure: " << exposure_time_);
    RCLCPP_INFO_STREAM(logger_, "Image size: " << image_width_ << "x" << image_height_);
    RCLCPP_INFO_STREAM(logger_, "Pixel format: " << pixel_format_);
    RCLCPP_INFO_STREAM(logger_, "Calibration file: " << (calibration_file_.empty() ? "(none)" : calibration_file_));
    RCLCPP_INFO_STREAM(logger_, "Trigger mode: " << trigger_mode_);
    // RCLCPP_INFO_STREAM(logger_, "Batch size: " << batch_size_);
    RCLCPP_INFO_STREAM(logger_, "User set selector: " << user_set_selector_);
    RCLCPP_INFO_STREAM(logger_, "Black level: " << black_level_);
    RCLCPP_INFO_STREAM(logger_, "Gamma: " << gamma_);
    RCLCPP_INFO_STREAM(logger_, "Acquisition frame rate: " << acquisition_frame_rate_);
    RCLCPP_INFO_STREAM(logger_, "Shutter mode: " << shutter_mode_);
    RCLCPP_INFO_STREAM(logger_, "PGI mode: " << (pgi_mode_ ? "on" : "off"));
    RCLCPP_INFO_STREAM(logger_, "Demosaicing mode: " << demosaicing_mode_);
    RCLCPP_INFO_STREAM(logger_, "Noise reduction: " << noise_reduction_);
}

void BaslerAceRos2Wrapper::applyCameraConfig()
{
    std::lock_guard<std::recursive_mutex> lock(grab_mutex_);
    camera_->stopGrabbingSafe();

    if (gain_ > 0.0) {
        double reachedGain = 0;
        camera_->setGain(gain_, reachedGain);
    }

    if (exposure_time_ > 0.0) {
        double reachedExp = 0;
        camera_->setExposure(exposure_time_, reachedExp);
    }

    if (black_level_ > 0) {
        camera_->setBlackLevel(black_level_);
    }

    if (gamma_ > 0.0 && gamma_ != 1.0f) {
        float reachedGamma = 0;
        camera_->setGamma(gamma_, reachedGamma);
    }

    camera_->setPixelFormat(pixel_format_, false);

    int reachedW = 0, reachedH = 0;
    camera_->setImageSize(image_width_, image_height_, reachedW, reachedH, false);

    try {
        camera_->getCamera().AcquisitionMode.SetValue(Basler_UniversalCameraParams::AcquisitionMode_Continuous);
    } catch (const Pylon::GenericException& e) {
        RCLCPP_WARN_STREAM(logger_, "AcquisitionMode: " << e.GetDescription());
    }
    camera_->setAcquisitionMode(acquisition_mode_);
    camera_->setAcquisitionBurstFrameCount(acquisition_burst_frame_count_);
    camera_->setAcquisitionFrameRateEnabled(acquisition_frame_rate_enabled_);
    camera_->setAcquisitionFrameRate(acquisition_frame_rate_);

    camera_->setShutterMode(shutter_mode_);
    camera_->setPgiMode(pgi_mode_);
    camera_->setDemosaicingMode(demosaicing_mode_);
    camera_->setNoiseReduction(noise_reduction_);

    // Apply the 5 targeted robust setters
    setUserSetSelector(user_set_selector_);
    setTriggerSelector(trigger_selector_);
    setTriggerSource(trigger_source_);
    setTriggerMode(trigger_mode_);
    setGrabbingStrategy(grabbing_strategy_);

    // // enable chunk data to get frame counter, timestamp
    // if (!camera_->camera_.ChunkModeActive.TrySetValue( true ))
    // {
    //     RCLCPP_WARN_STREAM(logger_, "The camera doesn't support chunk features");
    // }

    // // Enable time stamp chunks.
    // camera_->camera_.ChunkSelector.SetValue( ChunkSelector_Timestamp );
    // camera_->camera_.ChunkEnable.SetValue( true );

    // // Enable frame counter chunks?
    // if (camera_->camera_.ChunkSelector.TrySetValue( ChunkSelector_Framecounter ))
    // {
    //     // USB camera devices provide generic counters.
    //     // An explicit FrameCounter value is not provided by USB camera devices.
    //     // Enable frame counter chunks.
    //     camera_->camera_.ChunkEnable.SetValue( true );
    // }

    // // Enable CRC checksum chunks.
    // camera_->camera_.ChunkSelector.SetValue( ChunkSelector_PayloadCRC16 );
    // camera_->camera_.ChunkEnable.SetValue( true );

    // Restart grabbing after all parameters are applied.
    camera_->restartGrabbing(grabbing_strategy_);
}

std::string BaslerAceRos2Wrapper::pixelFormatToRosEncoding(const std::string& pf)
{
    if (pf == "BayerRG8")   return "bayer_rggb8";
    if (pf == "BayerGR8")   return "bayer_grbg8";
    if (pf == "BayerGB8")   return "bayer_gbrg8";
    if (pf == "BayerRG16")  return "bayer_rggb16";
    if (pf == "BayerGR16")  return "bayer_grbg16";
    if (pf == "BayerGB16")  return "bayer_gbrg16";
    if (pf == "BGR8packed" || pf == "BGR8") return "bgr8";
    if (pf == "RGB8packed" || pf == "RGB8") return "rgb8";
    if (pf == "Mono8")      return "mono8";
    if (pf == "Mono16")     return "mono16";
    return pf;
}

bool BaslerAceRos2Wrapper::isReady() const {
    return camera_ != nullptr;
}

std::string BaslerAceRos2Wrapper::setTriggerSource(int source) {
    std::lock_guard<std::recursive_mutex> lock(grab_mutex_);
    if (!isReady()) return "camera not ready";
    trigger_source_ = source;
    bool ok = camera_->setTriggerSource(source);
    return ok ? "done" : "failed to set trigger source";
}

std::string BaslerAceRos2Wrapper::setTriggerMode(bool mode) {
    std::lock_guard<std::recursive_mutex> lock(grab_mutex_);
    if (!isReady()) return "camera not ready";
    trigger_mode_ = mode;
    bool ok = camera_->setTriggerMode(mode);
    return ok ? "done" : "failed to set trigger mode";
}

std::string BaslerAceRos2Wrapper::setTriggerSelector(int selector) {
    std::lock_guard<std::recursive_mutex> lock(grab_mutex_);
    if (!isReady()) return "camera not ready";
    trigger_selector_ = selector;
    bool ok = camera_->setTriggerSelector(selector);
    return ok ? "done" : "failed to set trigger selector";
}

std::string BaslerAceRos2Wrapper::setGrabbingStrategy(int strategy) {
    std::lock_guard<std::recursive_mutex> lock(grab_mutex_);
    if (!isReady()) return "camera not ready";
    grabbing_strategy_ = strategy;
    camera_->stopGrabbingSafe();
    camera_->restartGrabbing(grabbing_strategy_);
    return "done";
}

std::string BaslerAceRos2Wrapper::setUserSetSelector(int selector) {
    std::lock_guard<std::recursive_mutex> lock(grab_mutex_);
    if (!isReady()) return "camera not ready";
    user_set_selector_ = selector;
    camera_->setUserSetSelector(selector);
    return "done";
}

void BaslerAceRos2Wrapper::setTriggerSourceCb(const std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Request> req,
                        std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Response> res) {
    res->message = setTriggerSource(req->value);
    res->success = (res->message == "done");
}

void BaslerAceRos2Wrapper::setTriggerModeCb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    res->message = setTriggerMode(req->data);
    res->success = (res->message == "done");
}

void BaslerAceRos2Wrapper::setTriggerSelectorCb(const std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Request> req,
                          std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Response> res) {
    res->message = setTriggerSelector(req->value);
    res->success = (res->message == "done");
}

void BaslerAceRos2Wrapper::setGrabbingStrategyCb(const std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Request> req,
                           std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Response> res) {
    res->message = setGrabbingStrategy(req->value);
    res->success = (res->message == "done");
}

void BaslerAceRos2Wrapper::setUserSetSelectorCb(const std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Request> req,
                          std::shared_ptr<pylon_ros2_camera_interfaces::srv::SetIntegerValue::Response> res) {
    res->message = setUserSetSelector(req->value);
    res->success = (res->message == "done");
}

}  // namespace suas_seed_case_tracking
