#pragma once

#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

namespace suas_seed_case_tracking {

class BaslerCam {
public:
    explicit BaslerCam(const std::string& serial_number);
    ~BaslerCam();

    BaslerCam(const BaslerCam&) = delete;
    BaslerCam& operator=(const BaslerCam&) = delete;

    bool setExposure(double target_us, double& reached_us);
    bool setGain(double target_db, double& reached_db);
    bool setImageSize(int width, int height, int& reached_w, int& reached_h, bool restart = true);
    bool setPixelFormat(const std::string& pixel_format, bool restart = true);
    bool setBlackLevel(int value);
    bool setGamma(float target, float& reached);
    bool setAcquisitionMode(int mode);
    bool setAcquisitionBurstFrameCount(int burst_count);
    bool setAcquisitionFrameRateEnabled(bool enabled);
    bool setAcquisitionFrameRate(int frame_rate);
    bool setTriggerMode(bool enable);
    bool setTriggerSource(int source);
    bool setTriggerSelector(int selector);
    bool setShutterMode(int mode);
    bool setPgiMode(bool on);
    bool setDemosaicingMode(int mode);
    bool setNoiseReduction(float value);
    bool setUserSetSelector(int selector);

    void overrideExposure(double exposure);
    double getFrameRate() const;
    double getExposureTime() const;
    double autoGain();
    double getGain() const;

    void open();
    std::vector<cv::Mat> capture(int num_imgs);
    cv::Mat captureOne();
    void captureAndSave(int num_imgs, const std::string& location);
    void close();

    Pylon::CBaslerUniversalInstantCamera& getCamera();
    const Pylon::CImageFormatConverter& getConverter() const;
    bool isGrabbing() const { return is_grabbing_; }

    cv::Mat convertToMat(const Pylon::CGrabResultPtr& ptrGrabResult);

    void setupConverter();
    void stopGrabbingSafe();
    void restartGrabbing(int strategy = 0);

private:
    Pylon::CBaslerUniversalInstantCamera camera_;
    Pylon::CImageFormatConverter converter_;
    bool is_grabbing_ = false;
};

}  // namespace suas_seed_case_tracking
