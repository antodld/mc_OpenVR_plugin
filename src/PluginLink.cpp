// #include "../include/OpenVRPlugin/OpenVRDataPublisher.h"
#include "../include/OpenVRPlugin/OpenVRData.h"
#include "../include/OpenVRPlugin/boost_serialization.h"
#include <thread>
#include <yaml-cpp/yaml.h>
#include <UDPDataLink/Publisher.h>

int main()
{

    OpenVRData openVRData;
    UDPDataLink::Publisher<std::map<std::string,vr::TrackedDevicePose_t>> sender;

    YAML::Node config = YAML::LoadFile("etc/OpenVRPlugin.yaml");

    if (!config["distantData"]) 
    {
        std::cout << "ERROR : No server parameters provided in the configuration file " << std::endl;
        return 1;
    }
    if(!config["distantData"]["ip"])
    {
        std::cout << "ERROR : no IP provided" <<std::endl;
        return 2;
    }
    if(!config["distantData"]["port"])
    {
        std::cout << "ERROR : no port number provided" <<std::endl;
        return 3;
    }

    const std::string ip = config["distantData"]["ip"].as<std::string>();
    const int port = config["distantData"]["port"].as<int>();
    sender.create(&ip[0],port);
    openVRData.init();
    
    std::cout << "Start sending data over IP :" << ip << " on port : " << port << std::endl;
    while(true)
    {
        std::map<std::string,vr::TrackedDevicePose_t> devicesData = openVRData.getDevicesData();
        openVRData.update(devicesData);
        sender.update_data(devicesData);
        // std::cout << "connected " << devicesData.begin()->first << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }

    return 0;


}