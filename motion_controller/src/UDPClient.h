#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <boost/array.hpp>
#include <boost/asio.hpp>

constexpr char MOTION_DATA_RIGHT = 1;
constexpr char MOTION_DATA_LEFT = 2;

using boost::asio::ip::udp;

class UDPClient {
public:
    UDPClient(std::string ip_mcr, std::string ip_mcl, unsigned short remote_port);

    void requestData();
    std::vector<char> readData(char data_type);

private:
    const std::string IP_MCR;
    const std::string IP_MCL;
    const unsigned short REMOTE_PORT;
    boost::asio::io_context io_context;
    udp::socket socket;
};

#endif
