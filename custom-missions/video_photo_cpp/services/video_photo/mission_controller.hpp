#pragma once

#include <airsdk/airsdk.hpp>
#include <arsdk/camera2.pb-c.h>
#include <arsdk/camera2.pb.h>
#include <msghub_utils.h>

class MissionController {
private:
    airsdk::control::ControlInterface mControlItf;
    int mVideoPhotoCurrentState;
    bool hasAlreadyHovered;

public:
    MissionController(pomp::Loop &loop);
    ~MissionController() = default;

    int start();
    void setVideoPhotoCurrentState(int state);

    int cmdFcamStartPhoto();
    int cmdFcamSetConfigPhoto();

    int onCmdReceived(const arsdk_cmd *cmd);

    void reactInSmToFlyingStateChange(int32_t state);
    void reactToEventState(arsdk::camera::Event *evt);
    void reactToEventStateConfigFieldNumber(arsdk::camera::Event *evt);
    void reactInSmToCameraMode(arsdk::camera::CameraMode mode);
    void reactInSmToEventPhoto(arsdk::camera::Event *evt);
};
