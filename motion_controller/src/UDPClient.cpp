#include "UDPClient.h"


UDPClient::UDPClient(
    std::string ip_mc,
    unsigned short udp_port,
    boost::asio::io_service& io_service)
: socket(io_service, udp::endpoint(udp::v4(), udp_port))
, remote_endpoint(boost::asio::ip::address::from_string(ip_mc), udp_port)
{
    recv_buf.assign(0);
    startReceive();
}


void UDPClient::startReceive() {
    socket.async_receive_from(
        boost::asio::buffer(recv_buf),
        remote_endpoint,
        boost::bind(
            &UDPClient::handleReceive,
            this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred
        )
    );
}


void UDPClient::handleReceive(const boost::system::error_code& error, std::size_t bytes_transferred) {
    startReceive();
}


void UDPClient::requestData() {
    boost::array<char, 1> send_buf  = {{ 0 }};
    socket.send_to(boost::asio::buffer(send_buf), remote_endpoint);
}


std::vector<char> UDPClient::readData() {
    std::vector<char> data(3);
    data[0] = recv_buf[0];
    data[1] = recv_buf[1];
    data[2] = recv_buf[2];
    recv_buf.assign(0);
    return data;
}
