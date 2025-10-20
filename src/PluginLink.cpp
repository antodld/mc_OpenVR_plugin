// #include "../include/OpenVRPlugin/OpenVRDataPublisher.h"
#include "../include/OpenVRPlugin/OpenVRData.h"
#include "../include/OpenVRPlugin/boost_serialization.h"
#include <UDPDataLink/Publisher.h>
#include <thread>
#include <yaml-cpp/yaml.h>

int configFromYAML(std::string & ip, int & port)
{
  YAML::Node config = YAML::LoadFile("etc/OpenVRPlugin.yaml");

  if(!config["OpenVRPlugin"]["distantData"])
  {
    std::cout << "ERROR : No server parameters provided in the configuration file " << std::endl;
    return 1;
  }
  if(!config["OpenVRPlugin"]["distantData"]["ip"])
  {
    std::cout << "ERROR : no IP provided" << std::endl;
    return 2;
  }
  if(!config["OpenVRPlugin"]["distantData"]["port"])
  {
    std::cout << "ERROR : no port number provided" << std::endl;
    return 3;
  }

  ip = config["OpenVRPlugin"]["distantData"]["ip"].as<std::string>();
  port = config["OpenVRPlugin"]["distantData"]["port"].as<int>();
  return 0;
}

int configFromArg(std::string & ip, int & port, int argc, char * argv[])
{
  for(int i = 0; i < argc; i++)
  {
    if(strcmp(argv[i], "--h") == 0)
    {
      ip = argv[i + 1];
    }
    if(strcmp(argv[i], "--p") == 0)
    {
      port = std::stoi(argv[i + 1]);
    }
    if(strcmp(argv[i], "--help") == 0)
    {
      std::cout << "./PluginLink --h [host] --p [port]" << std::endl;
      return 4;
    }
  }
  if(ip == "")
  {
    std::cout << "ERROR : no IP provided" << std::endl;
    return 2;
  }
  if(port == 0)
  {
    std::cout << "ERROR : no port number provided" << std::endl;
    return 3;
  }
  return 0;
}

int main(int argc, char * argv[])
{

  OpenVRData openVRData;
  std::string ip = "";
  int port = 0;
  std::cout << argc << std::endl;
  if(argc == 1)
  {
    const int res = configFromYAML(ip, port);
    if(res != 0)
    {
      return res;
    }
  }
  else
  {
    const int res = configFromArg(ip, port, argc, argv);
    if(res != 0)
    {
      return res;
    }
  }

  UDPDataLink::Publisher<std::map<std::string, vr::TrackedDevicePose_t>> sender(port);
  sender.start_reception();
  openVRData.init();

  std::cout << "Start sending data over IP :" << ip << " on port : " << port << std::endl;
  while(true)
  {
    std::map<std::string, vr::TrackedDevicePose_t> devicesData = openVRData.getDevicesData();
    openVRData.update(devicesData);
    sender.update_data(devicesData);
    // std::cout << "connected " << devicesData.begin()->first << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
