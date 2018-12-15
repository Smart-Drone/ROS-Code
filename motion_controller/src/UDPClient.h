#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>

constexpr char MOTION_DATA = 1;
constexpr char TAKEOFF_CMD = 2;
constexpr char LAND_CMD = 3;
constexpr char RESET_CMD = 4;

using boost::asio::ip::udp;

class UDPClient {
public:
    UDPClient(std::string ip_mc, unsigned short udp_port, boost::asio::io_service& io_service);

    void startReceive();
    void handleReceive(const boost::system::error_code& error, std::size_t bytes_transferred);
    void requestData();
    std::vector<char> readData();

private:
    udp::endpoint remote_endpoint;
    udp::socket socket;
    boost::array<char, 3> recv_buf;
};

#endif
