mc_OpenVR_plugin
==
This plugins provide an access to the OpenVR devices for [mc_rtc]

## Dependencies

[mc_rtc](https://github.com/jrl-umi3218/mc_rtc) (for control computer)

[OpenVR](https://github.com/ValveSoftware/openvr)

[UDPDataLink](https://github.com/antodld/UDPDataLink) (On operator computer for remote devices access)

## Installation

In repo directory :
```shell
mkdir build && cd build
cmake ..
make
sudo make install
```
## Use
To enable the plugin add to your [mc_rtc] configuration file :
```shell
Plugins: [OpenVRPlugin]
```

Each devices are identified with their ID, they can be labeled with a name :
```shell
deviceMap: #[name ; ID]
- ["RightFoot","LHR-D756B287"]
```

You can access the pose or the velocity of a device in the controller by using the following function in the datastore
```cpp
  auto & poseByNameFunc = ctl.datastore().get<std::function<sva::PTransformd(std::string)>>("OpenVRPlugin::getPoseByName");
  sva::PTransformd X_0_device = poseByNameFunc(deviceName);

  auto & velByNameFunc = ctl.datastore().get<std::function<sva::MotionVecd(std::string)>>("OpenVRPlugin::getVelocityByName");
  sva::MotionVecd V_device = velByNameFunc(deviceName);

  auto & getDevicesId = ctl.datastore().get<std::function<std::vector<std::string>()>>("OpenVRPlugin::getDevicesId");
  std::vector<std::string> deviceIDList = getDevicesId();
```
The following function are also available:
```cpp
-"OpenVRPlugin::getPoseById"

-"OpenVRPlugin::getVelocityById"
```


By default, it is assumed SteamVR is running on the same computer as the one running the controller.

# Distant device connection
If the devices are connected to another computer the data will be streamed using UDP. Using the following executable :
```shell
cd build
./PluginLink
```
On the control computer, change the configuration file
```yaml
localData: false
distantData:
  port: 12338
  ip: 127.0.0.1
```
If the config file is modified, the project must be built again

# SteamVR running on Ubuntu
This was tested on Ubuntu 20.04 with steamVR 2.2.3
Make sure the vive tracker dongle are connected before.
Run the following script :
```shell
~/.steam/debian-installation/steamapps/common/SteamVR/bin/vrmonitor.sh
```
If your steamVR is installed somewhere else, you can locate it using the game properties on steam directly
