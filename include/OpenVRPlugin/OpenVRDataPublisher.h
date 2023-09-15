#pragma once
#ifndef SENDER_H
#define SENDER_H
#include "serialize.h"
#include <netinet/in.h>
#include <iostream>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include "boost_serialization.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <fcntl.h>

template <typename T>
struct OpenVRDataPublisher
{
public:
    OpenVRDataPublisher()
    {
    }
    void create(const char* ip, const int port)
    {
        // Create a UDP socket
        socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        // Set the socket to non-blocking mode
        int flags = fcntl(socket_, F_GETFL, 0);
        if (flags == -1) {
            std::cerr << "Error getting socket flags: " << strerror(errno) << std::endl;
            return ;
        }
        flags |= O_NONBLOCK;
        if (fcntl(socket_, F_SETFL, flags) == -1) {
            std::cerr << "Error setting socket to non-blocking mode: " << strerror(errno) << std::endl;
            return ;
        }
    
        server_port_ = port;
        serverAddr_.sin_family = AF_INET;
        serverAddr_.sin_port = htons(server_port_);
        inet_pton(AF_INET, ip, &(serverAddr_.sin_addr));
        
        std::cout << "PUB Port : " << serverAddr_.sin_port << std::endl;
        std::cout << "PUB IP : " << serverAddr_.sin_addr.s_addr << std::endl;

        connected_ = true;

        
    }
    void update_data(const T & data)
    {
        data_ = data;
        update();
    }
    bool connected()
    {
        return connected_;
    }
    T get()
    {
        return data_;
    }
    ~OpenVRDataPublisher()
    {
        // close the socket
        close(socket_);
    }

private:
    void update()
    {
        if(connected_)
        {
            SerializableClass<T> objToSend(data_, "Hello, Server!");
            sendSerializedObject(objToSend);
        }
    }
    void sendSerializedObject(const SerializableClass<T>& obj)
    {

        std::string serializedData = serializeObject(obj);
        const char* data = serializedData.c_str();
        size_t dataSize = serializedData.size();

        sendto(socket_, &dataSize, sizeof(dataSize), 0,
            (struct sockaddr*)&serverAddr_, sizeof(serverAddr_));
        sendto(socket_, data, dataSize, 0,
            (struct sockaddr*)&serverAddr_, sizeof(serverAddr_));

    }

    T data_;
    int server_port_ = 12345;
    int socket_;
    bool connected_ = false;
    struct sockaddr_in serverAddr_;

};
#endif 