#include "OpenVRPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_plugin
{

OpenVRPlugin::~OpenVRPlugin()
{
  threadOn_ = false;
  th_.join();
}

void OpenVRPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  vive_module_ = mc_rbdyn::RobotLoader::get_robot_module("vive_tracker");
  viveRobots_ = mc_rbdyn::Robots::make();

  config_.load(config);
  config_.load(controller.controller().config());
  const auto & plugin_config = config_("OpenVRPlugin");
  if(plugin_config.has("deviceMap"))
  {
    std::vector<std::vector<std::string>> map = plugin_config("deviceMap"); //[name ; ID]
    for(auto & device : map)
    {
      nameIdMap_[device[0]] = device[1];
      idNameMap_[device[1]] = device[0];
    }
  }
  else
  {
    mc_rtc::log::warning("[{}] No Devices name listed in configuration file, device would not be accessible by name",
                         "OpenVRPlugin");
  }
  if(plugin_config.has("sleep"))
  {
    plugin_config("sleep", sleepTime_);
  }
  plugin_config("localData", localData_);
  if(!localData_)
  {
    const auto & distantDataC = plugin_config("distantData");
    const uint16_t n_port = distantDataC("port");
    const uint16_t n_local_port = distantDataC("local_port", 0);
    const std::string ip = distantDataC("ip");
    const size_t max_packet_size = distantDataC("max_packet_size", DEFAULT_MAX_PACKET_SIZE);
    mc_rtc::log::info("Create UDP receiver on port: {}, ip: {}", n_port, ip);
    // 0 lets the OS pick a local port
    receiver_ = std::make_unique<Receiver>(ip, n_port, n_local_port, max_packet_size);
    receiver_->set_verbose(distantDataC("verbose", false));
    std::string msg = fmt::format("Hello from OpenVRPlugin");
    receiver_->send_data(reinterpret_cast<const uint8_t *>(msg.data()), msg.size());
    receiver_->start_reception();
  }
  else
  {
    data_.init();
  }
  threadOn_ = true;
  th_ = std::thread(&OpenVRPlugin::threadLoop, this);

  controller.controller().datastore().make_call("OpenVRPlugin::getPoseByName",
                                                [this](const std::string & name) -> sva::PTransformd
                                                { return getPoseByName(name); });
  controller.controller().datastore().make_call(
      "OpenVRPlugin::getPoseById", [this](const std::string & id) -> sva::PTransformd { return getPoseByID(id); });

  controller.controller().datastore().make_call("OpenVRPlugin::getVelocityByName",
                                                [this](const std::string & name) -> sva::MotionVecd
                                                { return getVelocityByName(name); });
  controller.controller().datastore().make_call(
      "OpenVRPlugin::getVelocityById", [this](std::string id) -> sva::MotionVecd { return getVelocityById(id); });

  controller.controller().datastore().make_call("OpenVRPlugin::getDevicesId",
                                                [this]() -> std::vector<std::string> { return getDevicesId(); });

  controller.controller().datastore().make_call(
      "OpenVRPlugin::deviceOnline", [this](const std::string & name) -> bool { return deviceOnline(name); });

  controller.controller().datastore().make_call("OpenVRPlugin::deviceHasName",
                                                [this](const std::string & id) -> bool { return deviceHasName(id); });

  mc_rtc::log::info("OpenVRPlugin::init called with configuration:\n{}", plugin_config.dump(true, true));
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
  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_);
    devicesData_ = devicesDataThread_;
  }

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
    {
      std::lock_guard<std::mutex> lk_copy_state(mutex_);
      data_.update(devicesDataThread_);
    }
  }
  else
  {
    if(receiver_)
    {
      std::map<std::string, vr::TrackedDevicePose_t> data;
      if(receiver_->get(data))
      {
        std::map<std::string, vr::TrackedDevicePose_t>::iterator it;
        {
          std::lock_guard<std::mutex> lk_copy_state(mutex_);
          for(it = data.begin(); it != data.end(); it++)
          {
            devicesDataThread_[it->first] = it->second;
          }
        }
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

  for(auto & id : devicesId)
  {
    std::string gui_name = "ID :" + id;
    if(deviceIdHasName(id))
    {
      gui_name += " " + deviceNameById(id);
    }
    if(!gui->hasElement({"OpenVRPlugin", "Devices"}, gui_name))
    {
      std::string name = deviceNameById(id);
      gui->addElement({"OpenVRPlugin", "Devices"},
                      mc_rtc::gui::Transform(gui_name, [this, id]() -> sva::PTransformd { return getPoseByID(id); }));
      auto & viveRobot = viveRobots_->load(name, *vive_module_);
      gui->addElement({"OpenVRPlugin", "Devices"}, mc_rtc::gui::Robot(name,
                                                                      [&viveRobot, this, id]() -> mc_rbdyn::Robot &
                                                                      {
                                                                        viveRobot.posW(getPoseByID(id));
                                                                        return viveRobot;
                                                                      }));
    }
  }
}

sva::PTransformd OpenVRPlugin::convertTransform(const vr::HmdMatrix34_t & matrix)
{
  Eigen::Vector3d t_0;
  t_0 << matrix.m[0][3], matrix.m[1][3], matrix.m[2][3];
  Eigen::Matrix3d R_1_0;
  R_1_0 << matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
      matrix.m[2][0], matrix.m[2][1], matrix.m[2][2];

  return sva::PTransformd(R_1_0.transpose(), t_0);
}

sva::MotionVecd OpenVRPlugin::convertVelocity(const vr::HmdVector3_t & angular, const vr::HmdVector3_t & linear)
{

  Eigen::Vector3d v_0;
  v_0 << linear.v[0], linear.v[1], linear.v[2];
  Eigen::Vector3d va_0;
  va_0 << angular.v[0], angular.v[1], angular.v[2];

  return sva::MotionVecd(va_0, v_0);
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
