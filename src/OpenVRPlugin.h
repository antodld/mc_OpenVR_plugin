/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include "../include/OpenVRPlugin/OpenVRData.h"
#include "../include/OpenVRPlugin/boost_serialization.h"
#include <UDPDataLink/Publisher.h>
#include <UDPDataLink/Receiver.h>
#include <iostream>
#include <map>
#include <openvr/openvr.h>

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

  const vr::TrackedDevicePose_t & getDeviceById(const std::string & id) const
  {
    const std::map<std::string, vr::TrackedDevicePose_t> & data = getDevicesData();

    if(data.count(id) != 0)
    {
      return data.at(id);
    }
    mc_rtc::log::warning("[{}] Device Id {} is not available", "OpenVRPlugin", id);
    return empty_device_;
  }

  const vr::TrackedDevicePose_t getDeviceByName(const std::string & name) const
  {

    std::string id = "";

    if(nameIdMap_.count(name) != 0)
    {
      id = nameIdMap_.at(name);
    }
    else
    {
      mc_rtc::log::warning("[{}] Device name {} is not available in configuration", "OpenVRPlugin", name);
      return empty_device_;
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
    return convertVelocity(device.vAngularVelocity, device.vVelocity);
  }

  sva::MotionVecd getVelocityById(const std::string & id)
  {
    const auto device = getDeviceByName(id);
    return convertVelocity(device.vAngularVelocity, device.vVelocity);
  }

  void listDevicesId() const
  {
    const std::map<std::string, vr::TrackedDevicePose_t> & data = getDevicesData();

    for(auto & device : data)
    {
      mc_rtc::log::info("ID : {}", device.first);
    }
  }

  std::vector<std::string> getDevicesId() const
  {
    std::vector<std::string> out;
    const std::map<std::string, vr::TrackedDevicePose_t> & data = getDevicesData();
    for(auto & device : data)
    {
      out.push_back(device.first);
    }
    return out;
  }

  std::string deviceIdByName(const std::string & name) const
  {
    if(nameIdMap_.count(name) != 0)
    {
      return nameIdMap_.at(name);
    }
    else
    {
      mc_rtc::log::error("[{}] device name {} does not exist", "OpenVRPlugin", name);
    }
    return "";
  }
  std::string deviceNameById(const std::string & id) const
  {
    if(idNameMap_.count(id) != 0)
    {
      return idNameMap_.at(id);
    }
    else
    {
      mc_rtc::log::error("[{}] device ID {} does not exist", "OpenVRPlugin", id);
    }
    return "";
  }

  bool deviceHasName(const std::string & name) const
  {
    return nameIdMap_.count(name) != 0;
  }

  bool deviceOnline(const std::string & name) const
  {
    if(!deviceHasName(name))
    {
      mc_rtc::log::error("[{}] device name {} does not exist", "OpenVRPlugin", name);
      return false;
    }
    const auto id = deviceIdByName(name);

    const std::map<std::string, vr::TrackedDevicePose_t> & data = getDevicesData();

    return data.count(id) != 0;
  }

  bool deviceIdHasName(const std::string & id) const
  {
    return idNameMap_.count(id) != 0;
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

  const std::map<std::string, vr::TrackedDevicePose_t> & getDevicesData() const
  {
    return devicesData_;
  }

  using Receiver = UDPDataLink::Receiver<std::map<std::string, vr::TrackedDevicePose_t>>;
  std::unique_ptr<Receiver> receiver_ = nullptr;

  OpenVRData data_;
  std::mutex mutex_;
  std::map<std::string, vr::TrackedDevicePose_t> devicesData_; // Map device state to its ID

  std::map<std::string, vr::TrackedDevicePose_t> devicesDataThread_; // Map device state to its ID

  vr::TrackedDevicePose_t empty_device_;

  std::map<std::string, std::string> nameIdMap_; // Map device Id to a name
  std::map<std::string, std::string> idNameMap_; // Map device name to a Id

  bool localData_ = true; // Wether on not steamVR is on the machine running the plugin
  bool threadOn_ = false;
  std::thread th_;
  int sleepTime_ = 30;

  mc_rtc::Configuration config_;
};

} // namespace mc_plugin
