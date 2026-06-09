#include "suas_seed_case_tracking/basler_ace.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace Pylon;
using namespace Basler_UniversalCameraParams;
using namespace std;

namespace suas_seed_case_tracking {

BaslerCam::BaslerCam(const std::string& serial_number)
{
    CTlFactory& tlf = CTlFactory::GetInstance();
    DeviceInfoList_t devices;
    tlf.EnumerateDevices(devices);

    int camera_idx = -1;
    for (size_t idx = 0; idx < devices.size(); ++idx) {
        if (devices[idx].GetSerialNumber() == serial_number.c_str()) {
            camera_idx = static_cast<int>(idx);
            break;
        }
    }

    if (camera_idx < 0) {
        throw runtime_error("Basler camera with serial number " + serial_number + " cannot be found");
    }

    camera_.Attach(tlf.CreateDevice(devices[camera_idx]));
    camera_.Open();

    // Switch to SoftwareControl so GenICam nodes become writable.
    // Cameras often restore from NVM with ReadoutControl which locks parameters.
    try {
        GenApi::INodeMap& nodeMap = camera_.GetNodeMap();
        GenApi::CEnumerationPtr fc(nodeMap.GetNode("FeatureControl"));
        if (GenApi::IsAvailable(fc)) {
            fc->FromString("Software");
        }
    } catch (const GenICam::GenericException& e) {
        cerr << "Warning: could not set FeatureControl to Software: " << e.GetDescription() << endl;
    }
  }

BaslerCam::~BaslerCam()
{
    if (is_grabbing_) {
        try {
            camera_.StopGrabbing();
        } catch (...) {}
    }
    try {
        if (camera_.IsOpen()) {
            camera_.Close();
        }
    } catch (...) {}
}

void BaslerCam::stopGrabbingSafe()
{
    if (is_grabbing_) {
        try { camera_.StopGrabbing(); } catch (...) {}
        is_grabbing_ = false;
    }
}

void BaslerCam::restartGrabbing(int strategy)
{
    if (!is_grabbing_) {
        try {
            Pylon::EGrabStrategy pylon_strategy = Pylon::GrabStrategy_OneByOne;
            if (strategy == 1) pylon_strategy = Pylon::GrabStrategy_LatestImageOnly;
            else if (strategy == 2) pylon_strategy = Pylon::GrabStrategy_LatestImages;
            
            camera_.StartGrabbing(pylon_strategy);
            is_grabbing_ = true;
        } catch (...) {}
    }
}

bool BaslerCam::setExposure(double target_us, double& reached_us)
{
    try {
        camera_.ExposureAuto.TrySetValue(ExposureAuto_Off);

        double exposure = target_us;
        double min_exp = camera_.ExposureTime.GetMin();
        double max_exp = camera_.ExposureTime.GetMax();

        if (exposure < min_exp) {
            cout << "Exposure clamped to min: " << min_exp << endl;
            exposure = min_exp;
        } else if (exposure > max_exp) {
            cout << "Exposure clamped to max: " << max_exp << endl;
            exposure = max_exp;
        }

        camera_.ExposureTime.SetValue(exposure);
        reached_us = camera_.ExposureTime.GetValue();
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setExposure failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setGain(double target_db, double& reached_db)
{
    try {
        camera_.GainAuto.TrySetValue(GainAuto_Off);

        double gain = target_db;
        double min_gain = camera_.Gain.GetMin();
        double max_gain = camera_.Gain.GetMax();

        if (gain < min_gain) {
            cout << "Gain clamped to min: " << min_gain << endl;
            gain = min_gain;
        } else if (gain > max_gain) {
            cout << "Gain clamped to max: " << max_gain << endl;
            gain = max_gain;
        }

        camera_.Gain.SetValue(gain);
        reached_db = camera_.Gain.GetValue();
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setGain failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setImageSize(int width, int height, int& reached_w, int& reached_h, bool restart)
{
    try {
        stopGrabbingSafe();

        int64_t w = width;
        int64_t h = height;

        int64_t max_w = camera_.WidthMax.GetValue();
        int64_t min_w = camera_.Width.GetMin();
        int64_t max_h = camera_.HeightMax.GetValue();
        int64_t min_h = camera_.Height.GetMin();
        int64_t w_inc = camera_.Width.GetInc();
        int64_t h_inc = camera_.Height.GetInc();

        w = std::max(min_w, std::min(max_w, w));
        h = std::max(min_h, std::min(max_h, h));
        w -= w % w_inc;
        h -= h % h_inc;

        camera_.OffsetX.SetValue(0);
        camera_.OffsetY.SetValue(0);
        camera_.Width.SetValue(w);
        camera_.Height.SetValue(h);

        camera_.OffsetX.SetValue((max_w - camera_.Width.GetValue()) / 2);
        camera_.OffsetY.SetValue((max_h - camera_.Height.GetValue()) / 2);

        reached_w = static_cast<int>(camera_.Width.GetValue());
        reached_h = static_cast<int>(camera_.Height.GetValue());

        if (restart) restartGrabbing();
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setImageSize failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setPixelFormat(const std::string& pixel_format, bool restart)
{
    try {
        GenApi::INodeMap& node_map = camera_.GetNodeMap();
        GenApi::CEnumerationPtr enc_ptr(node_map.GetNode("PixelFormat"));
        GenApi::NodeList_t features;
        enc_ptr->GetEntries(features);

        bool available = false;
        for (auto& feat : features) {
            if (GenApi::IsAvailable(feat)) {
                GenApi::CEnumEntryPtr entry(feat);
                if (entry->GetSymbolic().c_str() == pixel_format) {
                    available = true;
                    break;
                }
            }
        }

        if (!available) {
            cerr << "Pixel format '" << pixel_format << "' not available on this camera" << endl;
            return false;
        }

        stopGrabbingSafe();
        camera_.PixelFormat.SetValue(pixel_format.c_str());
        if (restart) restartGrabbing();
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setPixelFormat failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setBlackLevel(int value)
{
    try {
        if (GenApi::IsAvailable(camera_.BlackLevel)) {
            camera_.BlackLevel.SetValue(value);
            return true;
        }
        cerr << "BlackLevel not available on this camera" << endl;
        return false;
    } catch (const GenICam::GenericException& e) {
        cerr << "setBlackLevel failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setGamma(float target, float& reached)
{
    try {
        if (!GenApi::IsAvailable(camera_.Gamma)) {
            cerr << "Gamma not available on this camera" << endl;
            return false;
        }

        float gamma = target;
        float min_gamma = static_cast<float>(camera_.Gamma.GetMin());
        float max_gamma = static_cast<float>(camera_.Gamma.GetMax());

        if (gamma < min_gamma) {
            cout << "Gamma clamped to min: " << min_gamma << endl;
            gamma = min_gamma;
        } else if (gamma > max_gamma) {
            cout << "Gamma clamped to max: " << max_gamma << endl;
            gamma = max_gamma;
        }

        camera_.Gamma.SetValue(gamma);
        reached = static_cast<float>(camera_.Gamma.GetValue());
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setGamma failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setAcquisitionMode(int mode)
{
    try {
        if (mode == 0) camera_.AcquisitionMode.SetValue(AcquisitionMode_SingleFrame);
        else if (mode == 1) camera_.AcquisitionMode.SetValue(AcquisitionMode_Continuous);
        else {
            cerr << "Invalid acquisition mode: " << mode << endl;
            return false;
        }
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setAcquisitionMode failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setAcquisitionBurstFrameCount(int burst_count)
{
    try {
        if (GenApi::IsAvailable(camera_.AcquisitionBurstFrameCount)) {
            camera_.AcquisitionBurstFrameCount.SetValue(burst_count);
            return true;
        }
        cerr << "AcquisitionBurstFrameCount not available on this camera" << endl;
        return false;
    } catch (const GenICam::GenericException& e) {
        cerr << "setAcquisitionBurstFrameCount failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setAcquisitionFrameRateEnabled(bool enabled)
{
    try {
        if (GenApi::IsAvailable(camera_.AcquisitionFrameRateEnable)) {
            camera_.AcquisitionFrameRateEnable.SetValue(enabled);
            return true;
        }
        cerr << "AcquisitionFrameRateEnable not available on this camera" << endl;
        return false;
    } catch (const GenICam::GenericException& e) {
        cerr << "setAcquisitionFrameRateEnabled failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setAcquisitionFrameRate(int frame_rate)
{
    try {
        if (GenApi::IsAvailable(camera_.AcquisitionFrameRate)) {
            camera_.AcquisitionFrameRate.SetValue(frame_rate);
        } else {
            cerr << "Acquisition frame rate not available on this camera" << endl;
            return false;
        }
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setAcquisitionFrameRate failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setTriggerMode(bool enable)
{
    try {
        camera_.TriggerMode.SetValue(enable ? TriggerMode_On : TriggerMode_Off);
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setTriggerMode failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setTriggerSource(int source)
{
    try {
        if (source == 0) camera_.TriggerSource.SetValue(TriggerSource_Software);
        else if (source == 1) camera_.TriggerSource.SetValue(TriggerSource_Line1);
        else if (source == 2) camera_.TriggerSource.SetValue(TriggerSource_Line3);
        else if (source == 3) camera_.TriggerSource.SetValue(TriggerSource_Line4);
        else if (source == 4) camera_.TriggerSource.SetValue(TriggerSource_SoftwareSignal1);
        else if (source == 5) camera_.TriggerSource.SetValue(TriggerSource_SoftwareSignal2);
        else if (source == 6) camera_.TriggerSource.SetValue(TriggerSource_SoftwareSignal3);
        else {
            cerr << "Invalid trigger source: " << source << endl;
            cerr << "Valid options: 0=Software, 1=Line1, 2=Line3, 3=Line4, 4=Timer, 5=SoftwareSignal1, 6=SoftwareSignal2, 7=SoftwareSignal3" << endl;
            return false;
        }
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setTriggerSource failed: " << source << " - " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setTriggerSelector(int selector)
{
    try {
        if (selector == 0) camera_.TriggerSelector.SetValue(TriggerSelector_FrameStart);
        else if (selector == 1) camera_.TriggerSelector.SetValue(TriggerSelector_FrameBurstStart);
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setTriggerSelector failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setShutterMode(int mode)
{
    try {
        switch (mode) {
            case 0: camera_.ShutterMode.SetValue(ShutterMode_Rolling); break;
            case 1: camera_.ShutterMode.SetValue(ShutterMode_Global); break;
            case 2: camera_.ShutterMode.SetValue(ShutterMode_GlobalResetRelease); break;
            default: return false;
        }
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setShutterMode failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setPgiMode(bool on)
{
    try {
        if (GenApi::IsAvailable(camera_.PgiMode)) {
            camera_.PgiMode.SetValue(on ? PgiMode_On : PgiMode_Off);
            return true;
        }
        cerr << "PGI mode not available on this camera" << endl;
        return false;
    } catch (const GenICam::GenericException& e) {
        cerr << "setPgiMode failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setDemosaicingMode(int mode)
{
    try {
        if (GenApi::IsAvailable(camera_.DemosaicingMode)) {
            switch (mode) {
                case 0: camera_.DemosaicingMode.SetValue(DemosaicingMode_Simple); break;
                case 1: camera_.DemosaicingMode.SetValue(DemosaicingMode_BaslerPGI); break;
                default: return false;
            }
            return true;
        }
        cerr << "Demosaicing mode not available on this camera" << endl;
        return false;
    } catch (const GenICam::GenericException& e) {
        cerr << "setDemosaicingMode failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setNoiseReduction(float value)
{
    try {
        if (GenApi::IsAvailable(camera_.NoiseReduction)) {
            camera_.NoiseReduction.SetValue(value);
            return true;
        } else if (GenApi::IsAvailable(camera_.BslNoiseReduction)) {
            camera_.BslNoiseReduction.SetValue(value);
            return true;
        }
        cerr << "Noise reduction not available on this camera" << endl;
        return false;
    } catch (const GenICam::GenericException& e) {
        cerr << "setNoiseReduction failed: " << e.GetDescription() << endl;
        return false;
    }
}

bool BaslerCam::setUserSetSelector(int selector)
{
    try {
        switch (selector) {
            case 0: camera_.UserSetSelector.SetValue(UserSetSelector_Default); break;
            case 1: camera_.UserSetSelector.SetValue(UserSetSelector_UserSet1); break;
            case 2: camera_.UserSetSelector.SetValue(UserSetSelector_UserSet2); break;
            case 3: camera_.UserSetSelector.SetValue(UserSetSelector_UserSet3); break;
            default:
                cerr << "Invalid user set selector: " << selector << ", using Default" << endl;
                camera_.UserSetSelector.SetValue(UserSetSelector_Default);
                break;
        }
        camera_.UserSetLoad.Execute();

        // UserSetLoad may restore FeatureControl from saved config, re-locking nodes.
        // Force Software mode again so subsequent parameter writes succeed.
        try {
            GenApi::INodeMap& nodeMap = camera_.GetNodeMap();
            GenApi::CEnumerationPtr fc(nodeMap.GetNode("FeatureControl"));
            if (GenApi::IsAvailable(fc)) {
                fc->FromString("Software");
            }
        } catch (...) {}

        cout << "UserSetSelector: " << camera_.UserSetSelector.ToString().c_str() << " loaded" << endl;
        return true;
    } catch (const GenICam::GenericException& e) {
        cerr << "setUserSetSelector failed: " << e.GetDescription() << endl;
        return false;
    }
}

void BaslerCam::overrideExposure(double exposure)
{
    camera_.ExposureTime.SetValue(exposure);
}

double BaslerCam::getFrameRate() const
{
    return camera_.ResultingFrameRate.GetValue();
}

double BaslerCam::getExposureTime() const
{
    return camera_.ExposureTime.GetValue();
}

// FIXME: not used (Brennan MT auto gain example)
// double BaslerCam::autoGain()
// {
//     camera_.GainAuto.SetValue(GainAuto_Once);
//     camera_.AcquisitionMode.SetValue(AcquisitionMode_SingleFrame);
//     camera_.TriggerMode.SetValue(TriggerMode_Off);
//     camera_.AutoGainUpperLimit.SetValue(36);
//     camera_.AutoGainLowerLimit.SetValue(0);

//     open();
//     this_thread::sleep_for(chrono::milliseconds(500));
//     close();

//     camera_.GainAuto.SetValue(GainAuto_Off);
//     camera_.AcquisitionMode.SetValue(AcquisitionMode_Continuous);
//     camera_.TriggerMode.SetValue(TriggerMode_On);
//     camera_.TriggerSelector.SetValue(TriggerSelector_FrameStart);
//     camera_.TriggerSource.SetValue(TriggerSource_Line1);
//     camera_.ExposureMode.SetValue(ExposureMode_Timed);

//     return camera_.Gain.GetValue();
// }

double BaslerCam::getGain() const
{
    return camera_.Gain.GetValue();
}

void BaslerCam::setupConverter()
{
    converter_.OutputPixelFormat = PixelType_BGR8packed;
    converter_.OutputBitAlignment = OutputBitAlignment_MsbAligned;
}

void BaslerCam::open()
{
    // when camera opened at the start, used OneByOne strategy
    // this will then be replaced by restartGrabbing using params
    camera_.StartGrabbing(GrabStrategy_OneByOne);
    setupConverter();
    is_grabbing_ = true;
    this_thread::sleep_for(chrono::seconds(1));
}

cv::Mat BaslerCam::convertToMat(const CGrabResultPtr& ptrGrabResult)
{
    CPylonImage targetImage;
    converter_.Convert(targetImage, ptrGrabResult);

    return cv::Mat(
        static_cast<int>(targetImage.GetHeight()),
        static_cast<int>(targetImage.GetWidth()),
        CV_8UC3,
        const_cast<uint8_t*>(static_cast<const uint8_t*>(targetImage.GetBuffer()))
    ).clone();
}

std::vector<cv::Mat> BaslerCam::capture(int num_imgs)
{
    int fpc_count = 0;
    std::vector<cv::Mat> images;

    while (camera_.IsGrabbing()) {
        CGrabResultPtr ptrGrabResult;

        try {
            camera_.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
        } catch (...) {
            cout << "Not all FPS images captured" << endl;
            cout << fpc_count << endl;
            break;
        }

        if (ptrGrabResult->GrabSucceeded()) {
            images.push_back(convertToMat(ptrGrabResult));
        } else {
            cout << "Frame " << fpc_count << " failed to grab" << endl;
        }

        fpc_count++;
        if (fpc_count >= num_imgs) {
            break;
        }
    }

    cout << "Camera state: " << (camera_.IsGrabbing() ? "grabbing" : "stopped") << endl;
    return images;
}

cv::Mat BaslerCam::captureOne()
{
    CGrabResultPtr ptrGrabResult;

    try {
        camera_.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
    } catch (...) {
        cout << "Frame capture timed out" << endl;
        return cv::Mat();
    }

    if (ptrGrabResult->GrabSucceeded()) {
        return convertToMat(ptrGrabResult);
    }

    return cv::Mat();
}

void BaslerCam::captureAndSave(int num_imgs, const std::string& location)
{
    auto images = capture(num_imgs);

    for (size_t idx = 0; idx < images.size(); ++idx) {
        string filename = location + to_string(idx) + ".png";
        cv::imwrite(filename, images[idx]);
    }
}

void BaslerCam::close()
{
    camera_.StopGrabbing();
    is_grabbing_ = false;
}

Pylon::CBaslerUniversalInstantCamera& BaslerCam::getCamera()
{
    return camera_;
}

const Pylon::CImageFormatConverter& BaslerCam::getConverter() const
{
    return converter_;
}

}  // namespace suas_seed_case_tracking
