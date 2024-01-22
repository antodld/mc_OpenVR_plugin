/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <openvr/openvr.h>
#include <iostream>
#include <map>
#include "../include/OpenVRPlugin/OpenVRData.h"
#include "../include/OpenVRPlugin/boost_serialization.h"
#include <UDPDataLink/Publisher.h>
#include <UDPDataLink/Receiver.h>

namespace mc_plugin
{

struct OpenVRPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  void updateDeviceGUI(mc_control::MCGlobalController & controller);

  vr::TrackedDevicePose_t getDeviceById(const std::string & id)
  {
    std::map<std::string,vr::TrackedDevicePose_t> data = getDevicesData();
 
    if(data.count(id) != 0)
    {
      return data[id];
    }
    mc_rtc::log::warning("[{}] Device Id {} is not available","OpenVRPlugin",id);   
    return vr::TrackedDevicePose_t();
  }

  vr::TrackedDevicePose_t getDeviceByName(const std::string & name)
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

    return getDeviceById(id);
  
  }

  sva::PTransformd getPoseByName(const std::string & name)
  {
    return convertTransform(getDeviceByName(name).mDeviceToAbsoluteTracking);
  }

  sva::PTransformd getPoseByID(const std::string & id)
  {
    return convertTransform(getDeviceById(id).mDeviceToAbsoluteTracking);
  }

  sva::MotionVecd getVelocityByName(const std::string & name)
  {
    auto device = getDeviceByName(name);
    return convertVelocity(device.vAngularVelocity,device.vVelocity);
  }

  sva::MotionVecd getVelocityById(const std::string & id)
  {
    auto device = getDeviceByName(id);
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

  std::vector<std::string> getDevicesId()
  {
    std::vector<std::string> out;
    std::map<std::string,vr::TrackedDevicePose_t> data = getDevicesData();
    std::map<std::string,vr::TrackedDevicePose_t>::iterator it;
    for (it = data.begin(); it != data.end(); it++)
    {
      out.push_back(it->first);
    }  
    return out;
  }

  std::string deviceIdByName(const std::string & name)
  {
    if(nameIdMap_.count(name) != 0)
    {
      return nameIdMap_[name];
    }
    else
    {
      mc_rtc::log::error("[{}] device name {} does not exist","OpenVRPlugin",name);
    }
    return "";
  }
  std::string deviceNameById(const std::string & id)
  {
    if(idNameMap_.count(id) != 0)
    {
      return idNameMap_[id];
    }
    else
    {
      mc_rtc::log::error("[{}] device ID {} does not exist","OpenVRPlugin",id);
    }
    return "";
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

  UDPDataLink::Receiver<std::map<std::string,vr::TrackedDevicePose_t>> receiver_;

  OpenVRData data_;
  std::mutex mutex_;
  std::map<std::string,vr::TrackedDevicePose_t> devicesData_; //Map device state to its ID
  std::map<std::string,std::string> nameIdMap_; //Map device Id to a name
  std::map<std::string,std::string> idNameMap_; //Map device name to a Id


  bool localData_ = true; //Wether on not steamVR is on the machine running the plugin
  bool threadOn_ = false;
  std::thread th_;
  int sleepTime_ = 30;

  mc_rtc::Configuration config_;


};

} // namespace mc_plugin
