#include "UDPClient.h"

UDPClient::UDPClient(std::string remote_ip, unsigned short remote_port)
: REMOTE_IP(remote_ip)
, REMOTE_PORT(remote_port)
, request_sent(false){

}


std::string UDPClient::requestData() {
    udp::endpoint receiver_endpoint;
    boost::asio::ip::address ip = boost::asio::ip::address::from_string(REMOTE_IP);
    receiver_endpoint.address(ip);
    receiver_endpoint.port(REMOTE_PORT);

    boost::asio::io_context io_context;
    udp::socket socket(io_context);
    socket.open(udp::v4());

    boost::array<char, 1> send_buf  = {{ 0 }};
    socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

    boost::array<char, 128> recv_buf;
    udp::endpoint sender_endpoint;
    size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);

    std::string sensor_data;
    std::copy(recv_buf.begin(), recv_buf.end(), std::back_inserter(sensor_data));

    return sensor_data;
}
