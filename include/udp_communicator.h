#pragma once
#include <string>
#include <functional>
#include <thread> // for std::thread, which is used in the implementation file
#include <atomic> // same as above
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "message_types.h"

class UDPCommunicator {
    public:
        // I don't know how to use it now;
        using MessageCallback = std::function<void(const std::string&, const json&, double)>;

        UDPCommunicator(const std::string& local_ip, int port,
                        const std::string& dest_ip, int dest_port);
        ~UDPCommunicator();

        bool start();
        void stop();

        // advertise any topic of any type
        bool send_json(const json& j);
        void set_message_callback(MesssageCallback cb);

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
};