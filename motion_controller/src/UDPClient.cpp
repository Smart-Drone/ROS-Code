#include "UDPClient.h"


UDPClient::UDPClient(
    std::string ip_mcr,
    std::string ip_mcl,
    unsigned short remote_port)
: IP_MCR(ip_mcr)
, IP_MCL(ip_mcl)
, REMOTE_PORT(remote_port)
, io_context()
, socket(io_context) {
    socket.open(udp::v4());
    boost::asio::socket_base::receive_buffer_size option(3);
    socket.set_option(option);    
}


void UDPClient::requestData() {
    udp::endpoint endpoint_mcr;
    boost::asio::ip::address ip_mcr = boost::asio::ip::address::from_string(IP_MCR);
    endpoint_mcr.address(ip_mcr);
    endpoint_mcr.port(REMOTE_PORT);

    udp::endpoint endpoint_mcl;
    boost::asio::ip::address ip_mcl = boost::asio::ip::address::from_string(IP_MCL);
    endpoint_mcl.address(ip_mcl);
    endpoint_mcl.port(REMOTE_PORT);

    boost::array<char, 1> send_buf  = {{ 0 }};
    socket.send_to(boost::asio::buffer(send_buf), endpoint_mcr);
    socket.send_to(boost::asio::buffer(send_buf), endpoint_mcl);
}


std::vector<char> UDPClient::readData(char packet_type) {
    boost::array<char, 3> recv_buf;
    udp::endpoint sender_endpoint;

    bool packet_received = false;
    while(!packet_received) {
        socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
        if(recv_buf[0] == packet_type) {
            packet_received = true;
        }
    }

    std::vector<char> data(3);
    data[0] = recv_buf[0];
    data[1] = recv_buf[1];
    data[2] = recv_buf[2];

    return data;
}
