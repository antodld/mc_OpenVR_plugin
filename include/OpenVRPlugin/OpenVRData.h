#pragma once
#include <openvr/openvr.h>
#include <iostream>
#include <map>
#include <mutex>
#include <chrono>


struct OpenVRData
{
    void init()
    {
        vr::EVRInitError eError {vr::VRInitError_None};
        vr_pointer = VR_Init(&eError, vr::VRApplication_Background);
        if (eError != vr::VRInitError_None)
        {
            vr_pointer = NULL;
            std::cout << "Unable to init VR runtime: " << VR_GetVRInitErrorAsEnglishDescription(eError) << "\n";
            exit(EXIT_FAILURE);
        }
    }
    void update(std::map<std::string,vr::TrackedDevicePose_t> & devicesData);

    std::map<std::string,vr::TrackedDevicePose_t> getDevicesData()
    {
        return devicesData_;
    }

private:

  std::map<std::string,vr::TrackedDevicePose_t> devicesData_; //Map device state to its ID
  vr::IVRSystem* vr_pointer {NULL};
  
  
};

// struct OpenVRData
// {
//     std::map<std::string,vr::TrackedDevicePose_t> devicesData_; //Map device state to its ID
// }


