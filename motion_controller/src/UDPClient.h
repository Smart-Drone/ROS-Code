#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <iostream>

using boost::asio::ip::udp;

class UDPClient {
public:
    UDPClient(std::string remote_ip, unsigned short remote_port);
    std::string requestData();

private:
    const std::string REMOTE_IP;
    const unsigned short REMOTE_PORT;
    bool request_sent;
};

#endif
