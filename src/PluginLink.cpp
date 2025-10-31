// #include "../include/OpenVRPlugin/OpenVRDataPublisher.h"
#include "../include/OpenVRPlugin/OpenVRData.h"
#include "../include/OpenVRPlugin/boost_serialization.h"
#include <UDPDataLink/Publisher.h>
#include <thread>
#include <yaml-cpp/yaml.h>

int configFromYAML(std::string & ip, uint16_t & port, bool & verbose, size_t & max_packet_size)
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
  port = config["OpenVRPlugin"]["distantData"]["port"].as<uint16_t>();
  if(config["OpenVRPlugin"]["distantData"]["verbose"])
  {
    verbose = config["OpenVRPlugin"]["distantData"]["verbose"].as<bool>();
  }
  if(config["OpenVRPlugin"]["distantData"]["max_packet_size"])
  {
    max_packet_size = config["OpenVRPlugin"]["distantData"]["max_packet_size"].as<size_t>();
  }
  return 0;
}

int configFromArg(std::string & ip, uint16_t & port, int argc, char * argv[])
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
  uint16_t port = 0;
  bool verbose = false;
  auto max_packet_size = DEFAULT_MAX_PACKET_SIZE;
  std::cout << argc << std::endl;
  if(argc == 1)
  {
    const int res = configFromYAML(ip, port, verbose, max_packet_size);
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

  UDPDataLink::Publisher<std::map<std::string, vr::TrackedDevicePose_t>> sender(port, max_packet_size);
  sender.start_reception();
  openVRData.init();

  std::cout << "Start sending data over IP :" << ip << " on port : " << port << std::endl;
  while(true)
  {
    std::map<std::string, vr::TrackedDevicePose_t> devicesData = openVRData.getDevicesData();
    openVRData.update(devicesData);
    sender.update_data(devicesData);
    sender.set_verbose(verbose);
    // std::cout << "connected " << devicesData.begin()->first << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
