mc_OpenVR_plugin
==
This plugins provide an access to the OpenVR devices for [mc_rtc]  

## Dependencies 

[mc_rtc] 

[OpenVR] 

[UDPDataLink] (for remote devices access) 

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
```
Plugins: [OpenVRPlugin]
```

Each devices are identified with their ID, they can be labeled with a name :
```shell
deviceMap: #[name ; ID]
- ["RightFoot","LHR-D756B287"]
```

By default, it is assumed SteamVR is running on the same computer as the one running the controller. 

# Distant device connection
If the devices are connected to another computer the data will be streamed using UDP. Using the following exécutable :
```shell
cd build
./PluginLink
```
On the control computer, change the configuration file
```shell
localData: false
distantData:
  port: 12338
  ip: 127.0.0.1
