#include "OpenVRPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

OpenVRPlugin::~OpenVRPlugin()
{
  threadOn_ = false;
  th_.join(); 
}

void OpenVRPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{

  vr::EVRInitError eError {vr::VRInitError_None};
  vr_pointer = VR_Init(&eError, vr::VRApplication_Background);
  if (eError != vr::VRInitError_None) {
      vr_pointer = NULL;
      std::cout << "Unable to init VR runtime: " << VR_GetVRInitErrorAsEnglishDescription(eError) << "\n";
      exit(EXIT_FAILURE);
  }


  if(config.has("deviceMap"))
  {
    std::vector<std::vector<std::string>> map = config("deviceMap");
    for (auto & device:map )
    {
      nameIdMap_[device[0]] = device[1];
    }
  }
  else
  {
    mc_rtc::log::warning("[{}] No Devices name listed in configuration file, device would not be accessible by name","OpenVRPlugin");
  }
  if(config.has("sleep"))
  {
    config("sleep",sleepTime_);
  }

  threadOn_ = true;
  th_ = std::thread(&OpenVRPlugin::threadLoop, this);

  mc_rtc::log::info("OpenVRPlugin::init called with configuration:\n{}", config.dump(true, true));
}

void OpenVRPlugin::reset(mc_control::MCGlobalController & controller)
{
  threadOn_ = false;
  th_.join();
  threadOn_ = true;
  th_ = std::thread(&OpenVRPlugin::threadLoop, this);
  mc_rtc::log::info("OpenVRPlugin::reset called");
}

void OpenVRPlugin::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("Tracker test\n{}",getPose("RightArm").translation());

  mc_rtc::log::info("Tracker speed test\n{}",getVelocity("RightArm").angular());


}

void OpenVRPlugin::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("OpenVRPlugin::after");
}

void OpenVRPlugin::update()
{
  std::map<std::string,vr::TrackedDevicePose_t> devicesData = getDevicesData(); //Map device state to its ID

  for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
  {

    vr::VRControllerState_t state;
    if (vr_pointer->GetControllerState(unDevice, &state, sizeof(state)))
    {
      
      DeviceData device;
      device.device_indx_ = unDevice;
      device.device_class_ = vr_pointer->GetTrackedDeviceClass(unDevice);

      char serialNumber[13];
      vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, vr::Prop_SerialNumber_String, serialNumber, sizeof(serialNumber));

      vr::VRControllerState_t controllerState;
      vr::TrackedDevicePose_t trackedControllerPose;

      vr_pointer->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice,
                                            &controllerState, sizeof(controllerState),
                                            &trackedControllerPose);

      auto matrix = trackedControllerPose.mDeviceToAbsoluteTracking;
      Eigen::Vector3d t_0;
      t_0 << matrix.m[0][3],matrix.m[1][3],matrix.m[2][3];
      devicesData[serialNumber] = trackedControllerPose;

    }
  }

  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_);
    devicesData_ = devicesData;
    
  }

}

sva::PTransformd OpenVRPlugin::convertTransform(const vr::HmdMatrix34_t & matrix)
{
  Eigen::Vector3d t_0;
  t_0 << matrix.m[0][3],matrix.m[1][3],matrix.m[2][3];
  Eigen::Matrix3d R_1_0;
  R_1_0 << matrix.m[0][0],matrix.m[0][1],matrix.m[0][2],
           matrix.m[1][0],matrix.m[1][1],matrix.m[1][2],
           matrix.m[2][0],matrix.m[2][1],matrix.m[2][2];
  
  return sva::PTransformd(R_1_0.transpose(),t_0);

}

sva::MotionVecd OpenVRPlugin::convertVelocity(const vr::HmdVector3_t & angular, const vr::HmdVector3_t & linear)
{

  Eigen::Vector3d v_0;
  v_0 << linear.v[0],linear.v[1],linear.v[2];
  Eigen::Vector3d va_0;
  va_0 << angular.v[0],angular.v[1],angular.v[2];
  
  return sva::MotionVecd(va_0,v_0);

}


mc_control::GlobalPlugin::GlobalPluginConfiguration OpenVRPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("OpenVRPlugin", mc_plugin::OpenVRPlugin)
