#include "../include/OpenVRPlugin/OpenVRData.h"

void OpenVRData::update(std::map<std::string, vr::TrackedDevicePose_t> & devicesData)
{
  for(vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
  {

    vr::VRControllerState_t state;
    if(vr_pointer->GetControllerState(unDevice, &state, sizeof(state)))
    {

      char serialNumber[13];
      vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, vr::Prop_SerialNumber_String, serialNumber,
                                                     sizeof(serialNumber));

      vr::VRControllerState_t controllerState;
      vr::TrackedDevicePose_t trackedControllerPose;

      vr_pointer->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState,
                                             sizeof(controllerState), &trackedControllerPose);

      auto matrix = trackedControllerPose.mDeviceToAbsoluteTracking;

      devicesData[serialNumber] = trackedControllerPose;

      devicesData_ = devicesData;
    }
  }
}
