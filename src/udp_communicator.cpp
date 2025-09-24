#include "udp_communicator.h"
#include <iostream>
#include <cstring>
#include <unistd.h>

// 构造函数中初始化心跳相关变量
UDPCommunicator::UDPCommunicator(const std::string& local_ip, int port,
                                 const std::string& dest_ip, int dest_port)
    : local_ip_(local_ip), local_port_(port),
      dest_ip_(dest_ip), dest_port_(dest_port),
      running_(false), sock_(-1),
      heartbeat_enabled_(false), heartbeat_interval_ms_(5000) {
    // 初始化时间点
    auto now = std::chrono::steady_clock::now();
    last_received_.store(now);
    last_heartbeat_.store(now);
}

void UDPCommunicator::stop() {
    if (running_) {
        running_ = false;
        heartbeat_enabled_ = false;  // 停止心跳

        // shutdown the socket to unblock recvfrom
        if (sock_ >= 0) {
            shutdown(sock_, SHUT_RDWR);  // shutdown the socket to unblock recvfrom
        }
        
        // 等待所有线程结束
        if (recv_thread_.joinable()) {
            recv_thread_.join();  // wait for the receive thread to finish
        }
        if (heartbeat_thread_.joinable()) {
            heartbeat_thread_.join();
        }

        // now it's safe to close
        if (sock_ >= 0) {
            close(sock_);
            sock_ = -1;
        }
        
        std::cout << "UDP Communicator stopped." << std::endl;
    }
    else {
        std::cout << "UDP Communicator is not running." << std::endl;
    }
}

UDPCommunicator::~UDPCommunicator() {
    stop();
    // no need to close sock_ here, already done in stop()
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
    if (local_ip_ == "0.0.0.0") { // good choice for binding to all interfaces
        recv_addr_.sin_addr.s_addr = INADDR_ANY; // bind to all interfaces
        ROS_INFO("Binding to all interfaces.");
    } else {
        ROS_INFO("Binding to local IP: %s", local_ip_.c_str());
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
    recv_thread_ = std::thread(&UDPCommunicator::receive_loop, this);  // 修复：receiver_thread_ -> recv_thread_
    
    // 如果心跳已启用，启动心跳线程
    if (heartbeat_enabled_) {
        heartbeat_thread_ = std::thread(&UDPCommunicator::heartbeat_loop, this);
    }
    
    std::cout << "UDP Communicator started on " << local_ip_ << ":" << local_port_
              << " sending to " << dest_ip_ << ":" << dest_port_ << std::endl;
    return true; // good to go
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
            
            // 更新最后接收时间
            last_received_.store(std::chrono::steady_clock::now());
            
            try {
                json j = json::parse(buffer);
                
                // 处理心跳消息
                if (MessageTypes::is_heartbeat(j)) {
                    last_heartbeat_.store(std::chrono::steady_clock::now());
                    // 可以选择回应心跳或只是记录
                    continue;  // 不传递给用户回调
                }
                
                // 处理普通消息
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
            // if (running_) { // only report error if we are supposed to be running
            //     std::cerr << "Receive error." << std::endl;
            //     std::cerr << "OH NO! May be the drone is dead." << std::endl;
            //     /*
            //         may add other logic here, like restarting the socket or something
            //         but for now, i don't know, he he, ha ha
            //     */
            // }
            int error_code = errno;  // get the error_code
            switch (error_code) {
                case EAGAIN:
                case EWOULDBLOCK:
                    // no data available right now in non-blocking mode
                    continue;
                case EINTR:
                    // interrupted by signal
                    ROS_WARN("EINTR:Receive interrupted by signal, continuing...");
                    continue;
                case ECONNRESET:
                    ROS_WARN("ECONNRESET:Connection reset by peer.");
                    break;
                case ENETUNREACH:
                    ROS_WARN("ENETUNREACH:Network unreachable.");
                    break;
                case EBADF:
                    // otherwise, EBADF means the socket was closed unexpectedly
                    if (running_) {
                        ROS_ERROR("EBADF: Invalid socket descriptor while running.");
                        ROS_ERROR("This indicates a serious problem - socket was closed unexpectedly.");
                    }
                    running_ = false;
                    break;
                default:
                    ROS_ERROR("Receive error: %s (errno: %d)", strerror(error_code), error_code);
                    break;
            }
            /*
                may add other logic here, like restarting the socket or something
                but for now, i don't know, he he, ha ha
            */
        }
        // else recv_len == 0 means no data received; continue
    }
}

// 心跳控制方法
void UDPCommunicator::enable_heartbeat(int interval_ms) {
    heartbeat_interval_ms_ = interval_ms;
    heartbeat_enabled_ = true;
    
    // 如果已经在运行，启动心跳线程
    if (running_ && !heartbeat_thread_.joinable()) {
        heartbeat_thread_ = std::thread(&UDPCommunicator::heartbeat_loop, this);
    }
    
    ROS_INFO("Heartbeat enabled with %d ms interval", interval_ms);
}

void UDPCommunicator::disable_heartbeat() {
    heartbeat_enabled_ = false;
    ROS_INFO("Heartbeat disabled");
}

// 心跳发送循环
void UDPCommunicator::heartbeat_loop() {
    while (running_ && heartbeat_enabled_) {
        // 发送心跳包
        json heartbeat = MessageTypes::make_heartbeat_message();
        if (!send_json(heartbeat)) {
            ROS_WARN("Failed to send heartbeat");
        }
        
        // 检查对端是否存活
        if (!check_peer_alive()) {
            ROS_WARN("Peer seems offline - no response for %.1f seconds", 
                     heartbeat_interval_ms_ * 3.0 / 1000.0);
            /*
                add other logic here
            */
        }
        
        // 等待指定间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms_));
    }
}

// 检查对端存活状态
bool UDPCommunicator::check_peer_alive() {
    auto now = std::chrono::steady_clock::now();
    auto last_received = last_received_.load();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_received);
    
    // 如果超过 3 个心跳间隔没收到消息，认为对端可能离线
    return elapsed.count() < (heartbeat_interval_ms_ * 3);
}

