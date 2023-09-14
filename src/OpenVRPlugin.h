/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <openvr/openvr.h>
#include <iostream>
#include <map>

namespace mc_plugin
{

struct OpenVRPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  vr::TrackedDevicePose_t getDevice(const std::string & name)
  {

    std::map<std::string,std::string>::iterator it = nameIdMap_.find(name);
    std::string id = "";

    if(nameIdMap_.count(name) != 0)
    {
      id = nameIdMap_[name];
    }
    else
    {
      mc_rtc::log::warning("[{}] Device name {} is not available in configuration","OpenVRPlugin",name);
      return vr::TrackedDevicePose_t();
    }

    std::map<std::string,vr::TrackedDevicePose_t> data = getDevicesData();
 
    if(data.count(id) != 0)
    {
      return data[id];
    }
    mc_rtc::log::warning("[{}] Device Id {} is not available","OpenVRPlugin",id);
    return vr::TrackedDevicePose_t();
  
  }

  sva::PTransformd getPose(const std::string & name)
  {
    return convertTransform(getDevice(name).mDeviceToAbsoluteTracking);
  }

  sva::MotionVecd getVelocity(const std::string & name)
  {
    auto device = getDevice(name);
    return convertVelocity(device.vAngularVelocity,device.vVelocity);
  }

  void listDevicesId()
  {
    std::map<std::string,vr::TrackedDevicePose_t> data = getDevicesData();
    std::map<std::string,vr::TrackedDevicePose_t>::iterator it;
    for (it = data.begin(); it != data.end(); it++)
    {
      mc_rtc::log::info("ID : {}",it->first);
    }
  }

  ~OpenVRPlugin() override;

private:

  void update();

  void threadLoop()
  {
    while(threadOn_)
    {
      update();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime_));
    }
  }

  sva::PTransformd convertTransform(const vr::HmdMatrix34_t & matrix);
  sva::MotionVecd convertVelocity(const vr::HmdVector3_t & angular, const vr::HmdVector3_t & linear);

  std::map<std::string,vr::TrackedDevicePose_t> getDevicesData()
  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_);
    return devicesData_;
  }

  std::mutex mutex_;
  std::map<std::string,vr::TrackedDevicePose_t> devicesData_; //Map device state to its ID
  std::map<std::string,std::string> nameIdMap_; //Map device Id to a name
  vr::IVRSystem* vr_pointer {NULL};

  bool threadOn_ = false;
  std::thread th_;
  int sleepTime_ = 30;


};

} // namespace mc_plugin


struct DeviceData
{
    sva::PTransformd X_0_device_;
    sva::MotionVecd V_device_ = sva::MotionVecd::Zero();
    sva::MotionVecd A_device_ = sva::MotionVecd::Zero();
    vr::TrackedDeviceIndex_t device_indx_ = 0;
    vr::ETrackedDeviceClass device_class_ = vr::ETrackedDeviceClass::TrackedDeviceClass_Invalid;
    
};