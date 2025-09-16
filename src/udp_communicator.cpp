#include "udp_communicator.h"
#include <iostream>
#include <cstirng>
#include <unistd.h>

UDPCommunicator::UDPCommunicator(const std::string& local_ip, int port,
                                 const std::string& dest_ip, int dest_port)
    : local_ip_(local_ip), local_port_(port),
      dest_ip_(dest_ip), dest_port_(dest_port),
      running_(false), sock_(-1) {}

UDPCommunicator::~UDPCommunicator() {
    stop();
    if (sock_ >= 0) {
        close(sock_);
    }   // ensure socket is closed
}

bool UDPCommunicator::start() {
    // ensure not already running
    if (running_) {
        std::cerr << "UDPCommunicator is already running." << std::endl;
        return false;
    }

    sock_ = socket(AF_INET, SOCKET_DGRAM, 0);
    if (sock_ < 0) {
        std::cerr << "Failed to create socket." << std::endl;
        return false;
    }

    // setup destination address
    memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(dest_port_);
    inet_pton(AF_INET, dest_ip_.c_str(), &dest_addr_.sin_addr);

    // setup local address
    memset(&recv_addr_, 0, sizeof(recv_addr_));
    recv_addr_.sin_family = AF_INET;
    // recv_addr_.sin_addr.s_addr = inet_addr(local_ip_.c_str());
    if (local_ip_ == "0.0.0.0") {
        recv_addr_.sin_addr.s_addr = INADDR_ANY; // bind to all interfaces
        ROS::INFO("Binding to all interfaces.");
    } else {
        ROS::INFO("Binding to local IP: %s", local_ip_.c_str());
        recv_addr_.sin_addr.s_addr = inet_addr(local_ip_.c_str());
        if (recv_addr_.sin_addr.s_addr == INADDR_NONE) {
            std::cerr << "Invalid local IP address." << std::endl;
            close(sock_);
            sock_ = -1;
            return false;
        }
    }
    recv_addr_.sin_port = htons(local_port_);

    if (bind(sock_, (struct sockaddr*)&recv_addr_, sizeof(recv_addr_)) < 0) {
        std::cerr << "Failed to bind socket." << std::endl;
        close(sock_);
        sock_ = -1;
        return false;
    }

    running_ = true;
    receiver_thread_ = std::thread(&UDPCommunicator::receive_loop, this);
    std::cout << "UDP Communicator started on " << local_ip_ << ":" << local_port_
              << " sending to " << dest_ip_ << ":" << dest_port_ << std::endl;
    return true; // good to go
}

void UDPCommunicator::stop() {
    if (running_) {
        running_ = false;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
        std::cout << "UDP Communicator stopped." << std::endl;
    }
    else {
        std::cout << "UDP Communicator is not running." << std::endl;
    }
}


bool UDPCommunicator::send_json(const json& j) {
    if (sock_ < 0) {
        std::cerr << "OH NO! Socket is not initialized." << std::endl;
        return false;// oh no
    }

    std::string payload = j.dump();
    // ssize_t means signed size type
    ssize_t sent_bytes = sendto(sock_, payload.c_str(), payload.size(), 0,
                                (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
    return sent_bytes == (ssize_t)payload.size();
}


void UDPCommunicator::set_message_callback(MessageCallback cb) {
    message_callback_ = cb;
}

void UDPCommunicator::receive_loop() {
    char buffer[2048];
    while (running_) {
        socklen_t addr_len = sizeof(recv_addr_);
        // sizeof(buffer) - 1 to leave space for null-terminator ("\0")
        ssize_t recv_len = recvfrom(sock_, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&recv_addr_, &addr_len);
        if (recv_len > 0) {
            buffer[recv_len] = '\0'; // null-terminate
            try {
                json j = json::parse(buffer);
                std::string topic;
                std::string msg_type;
                json data;
                double timestamp;
                MessageTypes::parse_received(j, topic, msg_type, data, timestamp);
                if (message_callback_) {
                    message_callback_(topic, data, timestamp);
                }
            } catch (const std::exception& e) {
                std::cerr << "Failed to parse JSON: " << e.what() << std::endl;
            }
        } else if (recv_len < 0) {
            if (running_) { // only report error if we are supposed to be running
                std::cerr << "Receive error." << std::endl;
                std::cerr << "OH NO! May be the drone is dead." << std::endl;
                /*
                    may add other logic here, like restarting the socket or something
                    but for now, i don't know, he he, ha ha
                */
            }
        }
        // else recv_len == 0 means no data received; continue
    }
}

