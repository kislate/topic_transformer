#pragma once
#include <string>
#include <functional>
#include <thread> // for std::thread, which is used in the implementation file
#include <atomic> // same as above
#include <chrono> // for time_point in heartbeat functionality
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "message_types.h"

class UDPCommunicator {
    public:
        // I don't know how to use it now;
        // maybe i know now: it is a callback function type for received messages
        //                               ------2025-9-20
        using MessageCallback = std::function<void(const std::string&, const json&, double)>;


        // constructor: local IP and port to bind, destination IP and port to send to
        UDPCommunicator(const std::string& local_ip, int port,
                        const std::string& dest_ip, int dest_port);
        
        // destructor
        ~UDPCommunicator();

        // start the communicator: create socket, bind, and start receive thread
        bool start();
        void stop();

        // advertise any topic of any type
        bool send_json(const json& j);
        void set_message_callback(MessageCallback cb);  
        void enable_heartbeat(int interval_ms = 5000);  // 启用心跳，默认5秒
        void disable_heartbeat();

        private:
            void receive_loop();

            std::string local_ip_, dest_ip_;
            int local_port_, dest_port_;
            int sock_;
            // socket address
            struct sockaddr_in dest_addr_, recv_addr_;

            std::atomic<bool> running_;         // for controlling the receive loop
            std::thread recv_thread_;           // for receiving messages thread
            MessageCallback message_callback_;  // user-defined callback for received messages

            // 心跳相关
            void heartbeat_loop();                          // 心跳发送循环
            bool check_peer_alive();                        // 检查对端是否存活
            
            std::atomic<bool> heartbeat_enabled_;           // 心跳开关
            int heartbeat_interval_ms_;                     // 心跳间隔（毫秒）
            std::thread heartbeat_thread_;                  // 心跳线程
            
            std::atomic<std::chrono::steady_clock::time_point> last_received_;  // 最后收到消息时间
            std::atomic<std::chrono::steady_clock::time_point> last_heartbeat_; // 最后收到心跳时间
};