#include "OpenVRPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include<mc_rtc/gui/Transform.h>
#include<mc_rtc/gui/Label.h>

namespace mc_plugin
{

OpenVRPlugin::~OpenVRPlugin()
{
  threadOn_ = false;
  th_.join(); 
}

void OpenVRPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{

  if(config.has("deviceMap"))
  {
    std::vector<std::vector<std::string>> map = config("deviceMap"); //[name ; ID]
    for (auto & device:map )
    {
      nameIdMap_[device[0]] = device[1];
      idNameMap_[device[1]] = device[0];
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
  config("localData",localData_);
  if(!localData_)
  {
    const int n_port = config("distantData")("port");
    mc_rtc::log::info("create UDP receiver on port {}",n_port);
    receiver_.create(n_port);
  }
  else
  {
    data_.init();
  }
  threadOn_ = true;
  th_ = std::thread(&OpenVRPlugin::threadLoop, this);

  controller.controller().datastore().make_call(
    "OpenVRPlugin::getPoseByName", [this](std::string name) -> sva::PTransformd { return getPoseByName(name); });
  controller.controller().datastore().make_call(
    "OpenVRPlugin::getPoseById", [this](std::string id) -> sva::PTransformd { return getPoseByID(id); });
  
  controller.controller().datastore().make_call(
    "OpenVRPlugin::getVelocityByName", [this](std::string name) -> sva::MotionVecd { return getVelocityByName(name); });
  controller.controller().datastore().make_call(
    "OpenVRPlugin::getVelocityById", [this](std::string id) -> sva::MotionVecd { return getVelocityById(id); });
  
  controller.controller().datastore().make_call(
    "OpenVRPlugin::getDevicesId", [this]() -> std::vector<std::string> { return getDevicesId(); });


  mc_rtc::log::info("OpenVRPlugin::init called with configuration:\n{}", config.dump(true, true));
}

void OpenVRPlugin::reset(mc_control::MCGlobalController & controller)
{

  controller.controller().gui()->removeCategory({"OpenVRPlugin"});

  threadOn_ = false;
  th_.join();
  threadOn_ = true;
  th_ = std::thread(&OpenVRPlugin::threadLoop, this);
  
  mc_rtc::log::info("OpenVRPlugin::reset called");
}

void OpenVRPlugin::before(mc_control::MCGlobalController & ctl)
{
  updateDeviceGUI(ctl);
  // mc_rtc::log::info("Tracker test\n{}",getPoseByName("RightArm").translation());
  
}

void OpenVRPlugin::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("OpenVRPlugin::after");
}

void OpenVRPlugin::update()
{
  if(localData_)
  {
    std::map<std::string,vr::TrackedDevicePose_t> devicesData = getDevicesData(); //Map device state to its ID
    data_.update(devicesData);
    {
      std::lock_guard<std::mutex> lk_copy_state(mutex_);
      devicesData_ = devicesData;
      
    }
  }
  else
  {
    if(receiver_.connected())
    {
      std::map<std::string,vr::TrackedDevicePose_t> data;
      int data_available = 0;
      size_t count = 0;
      while(data_available == 0)
      {
        data_available = receiver_.receive();
        if(data_available != 0)
        {
          break;
        }
        count +=1;
      }
      receiver_.get(data);
      std::lock_guard<std::mutex> lk_copy_state(mutex_);
            
      std::map<std::string,vr::TrackedDevicePose_t>::iterator it;
      for (it = data.begin(); it != data.end(); it++)
      {
        devicesData_[it->first] = it->second;
      } 
    
    }
    else
    {
      mc_rtc::log::warning("[OpenVRPlugin] Data sender is offline");
    }
  }

}

void OpenVRPlugin::updateDeviceGUI(mc_control::MCGlobalController & controller)
{
  auto gui = controller.controller().gui();

  auto devicesId = getDevicesId();

  for(auto & id: devicesId)
  {
    if(!gui->hasElement({"OpenVRPlugin","Devices"},"ID :" + id))
    {
      std::string name = deviceNameById(id);
      gui->addElement({"OpenVRPlugin","Devices"}, mc_rtc::gui::Transform("ID :" + id,[this,id]() -> sva::PTransformd {return getPoseByID(id);}));
      gui->addElement({"OpenVRPlugin","Devices"}, mc_rtc::gui::Label("Associated Name : " + name,[this]() -> std::string {return "";}));
    }
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
