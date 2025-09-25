#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include "message_types.h"
#include "udp_communicator.h"
#include <map>
#include <vector>
#include <memory>  // 添加这个头文件

// drone_id variable (no use
int drone_id;
std::string topic_prefix_;

// set global udp_comm_
/*
set global udp_comm_ using smart pointer
*/
// UDPCommunicator udp_comm_;
std::unique_ptr<UDPCommunicator> udp_comm_;

// 消息类型枚举，方便switch使用
enum class MessageType {
    POSE,
    STRING,
    UNKNOWN
};

// 字符串到枚举的转换函数
MessageType getMessageType(const std::string& type_str) {
    if (type_str == "pose_stamped") return MessageType::POSE;
    if (type_str == "string") return MessageType::STRING;
    return MessageType::UNKNOWN;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "topic_transformer_node");
    ros::NodeHandle nh("~");
    
    // get parameters
    std::string local_ip_, dest_ip_;
    int local_port_, dest_port_;
    bool heartbeat_enabled_;
    int heartbeat_interval_ms_;
    int sub_topic_num = 0;

    nh.param<int>("drone_id", drone_id, 0);
    nh.param<std::string>("local_ip", local_ip_, "0.0.0.0");
    nh.param<int>("local_port", local_port_, 9000);
    nh.param<std::string>("dest_ip", dest_ip_, "100.67.3.30");
    nh.param<int>("dest_port", dest_port_, 9000);
    nh.param<bool>("heartbeat_enabled", heartbeat_enabled_, true);
    nh.param<int>("heartbeat_interval_ms", heartbeat_interval_ms_, 5000);
    nh.param<std::string>("topic_prefix", topic_prefix_, "drone_1");
    nh.param<int>("sub_topic_num", sub_topic_num, 1);

    // // udp communicator
    // udp_comm_ = UDPCommunicator(local_ip_, local_port_, dest_ip_, dest_port_);
    udp_comm_.reset(new UDPCommunicator(local_ip_, local_port_, dest_ip_, dest_port_));
    
    // 存储订阅器，防止被销毁
    std::vector<ros::Subscriber> subscribers;

    for (int i = 1; i <= sub_topic_num; i++) {
        std::string topic_param = "sub_topic_" + std::to_string(i) + "_name";
        std::string type_param = "sub_topic_" + std::to_string(i) + "_type";
        std::string sub_topic;
        std::string sub_topic_type;
        nh.param<std::string>(topic_param, sub_topic, "");
        nh.param<std::string>(type_param, sub_topic_type, "");
        
        if (sub_topic.empty() || sub_topic_type.empty()) {
            ROS_WARN("Sub topic %d name or type is empty, skipping...", i);
            continue;
        }
        
        MessageType msg_type = getMessageType(sub_topic_type);
        
        switch (msg_type) {
            case MessageType::POSE: {
                // 订阅PoseStamped类型
                auto callback = [sub_topic](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    json j = MessageTypes::make_pose_message(sub_topic, *msg);
                    ROS_INFO("Received pose message on topic %s: %s", sub_topic.c_str(), j.dump().c_str());
                    udp_comm_->send_json(j);  // 使用 -> 访问
                };
                
                subscribers.push_back(nh.subscribe<geometry_msgs::PoseStamped>(sub_topic, 10, callback));
                ROS_INFO("Created pose subscriber for topic: %s", sub_topic.c_str());
                break;
            }
            
            case MessageType::STRING: {
                // 订阅String类型
                auto callback = [sub_topic](const std_msgs::String::ConstPtr& msg) {
                    json j = MessageTypes::make_string_message(sub_topic, *msg);
                    ROS_INFO("Received string message on topic %s: %s", sub_topic.c_str(), j.dump().c_str());
                    udp_comm_->send_json(j);  // 使用 -> 访问
                };
                
                subscribers.push_back(nh.subscribe<std_msgs::String>(sub_topic, 10, callback));
                ROS_INFO("Created string subscriber for topic: %s", sub_topic.c_str());
                break;
            }
            
            case MessageType::UNKNOWN:
            default: {
                ROS_WARN("Unknown sub topic type: %s", sub_topic_type.c_str());
                break;
            }
        }
    }

    // publisher maps for different message types
    static std::map<std::string, ros::Publisher> pose_publishers;
    static std::map<std::string, ros::Publisher> string_publishers;

    // configure heartbeat BEFORE starting
    if (heartbeat_enabled_) {
        udp_comm_->enable_heartbeat(heartbeat_interval_ms_);  // 使用 -> 访问
        ROS_INFO("Heartbeat enabled with interval %d ms", heartbeat_interval_ms_);
    } else {
        udp_comm_->disable_heartbeat();  // 使用 -> 访问
        ROS_INFO("Heartbeat disabled");
    }

    // set message callback BEFORE starting
    udp_comm_->set_message_callback(  // 使用 -> 访问
        [&nh](const std::string& topic, const json& j, double timestamp) {
            if (MessageTypes::is_heartbeat(j)) {
                ROS_INFO("Received heartbeat from %s at %.3f", topic.c_str(), timestamp);
                return;
            } 
            ROS_INFO("Received message on topic %s at %.3f: %s",
                     topic.c_str(), timestamp, j.dump().c_str());
            
            // transform topic name by adding prefix
            std::string transformed_topic = topic_prefix_ + "/" + topic;
            
            // 获取消息类型并转换为枚举
            std::string msg_type_str = j.value("type", "unknown");
            MessageType msg_type = getMessageType(msg_type_str);
            
            // 使用switch处理不同消息类型
            switch (msg_type) {
                case MessageType::POSE: {
                    if (pose_publishers.find(transformed_topic) == pose_publishers.end()) {
                        pose_publishers[transformed_topic] = nh.advertise<geometry_msgs::PoseStamped>(transformed_topic, 10);
                        ROS_INFO("Created pose publisher for topic %s", transformed_topic.c_str());
                        ros::Duration(0.1).sleep();
                    }
                    
                    geometry_msgs::PoseStamped pose_msg;
                    auto& data = j["data"];
                    pose_msg.header.seq = data["header"]["seq"];
                    pose_msg.header.stamp = ros::Time(data["header"]["stamp"]);
                    pose_msg.header.frame_id = data["header"]["frame_id"];
                    auto& p = data["pose"]["position"];
                    auto& o = data["pose"]["orientation"];
                    pose_msg.pose.position.x = p["x"];
                    pose_msg.pose.position.y = p["y"];
                    pose_msg.pose.position.z = p["z"];
                    pose_msg.pose.orientation.x = o["x"];
                    pose_msg.pose.orientation.y = o["y"];
                    pose_msg.pose.orientation.z = o["z"];
                    pose_msg.pose.orientation.w = o["w"];
                    
                    pose_publishers[transformed_topic].publish(pose_msg);
                    ROS_INFO("Published pose message to %s", transformed_topic.c_str());
                    break;
                }
                
                case MessageType::STRING: {
                    if (string_publishers.find(transformed_topic) == string_publishers.end()) {
                        string_publishers[transformed_topic] = nh.advertise<std_msgs::String>(transformed_topic, 10);
                        ROS_INFO("Created string publisher for topic %s", transformed_topic.c_str());
                        ros::Duration(0.1).sleep();
                    }
                    
                    std_msgs::String str_msg;
                    str_msg.data = j["data"]["message"];
                    
                    string_publishers[transformed_topic].publish(str_msg);
                    ROS_INFO("Published string message to %s", transformed_topic.c_str());
                    break;
                }
                
                case MessageType::UNKNOWN:
                default: {
                    ROS_WARN("Unknown message type: %s", msg_type_str.c_str());
                    break;
                }
            }
        });
    
    // start UDP communicator AFTER all configurations
    if (!udp_comm_->start()) {  // 使用 -> 访问
        ROS_ERROR("Failed to start UDP Communicator");
        return -1;
    }

    ROS_INFO("UDP Communicator started on %s:%d sending to %s:%d",
             local_ip_.c_str(), local_port_, dest_ip_.c_str(), dest_port_);
    ROS_INFO("Created %lu subscribers for local topics", subscribers.size());
    
    ROS_INFO("Topic Transformer Node is running...");
    ros::spin();
    
    return 0;
}