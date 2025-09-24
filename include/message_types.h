// make json messages from ROS msgs, and parse received json messages

#pragma once
#include "json.hpp"
#include <ros/ros.h>  // 添加：为了使用 ros::Time
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <cstdlib>    // 添加：为了使用 rand() 函数

using json = nlohmann::json;

const std::string MSG_TYPE_STRING = "string";
const std::string MSG_TYPE_POSE = "pose_stamped";
const std::string MSG_TYPE_HEARTBEAT = "heartbeat"; // add heartbeat message
 
class MessageTypes {
    public:
        // heartbeat_message
        static json make_heartbeat_message() {
            json j;
            j["type"] = MSG_TYPE_HEARTBEAT;
            j["topic"] = "/heartbeat";
            j["timestamp"] = ros::Time::now().toSec();
            j["data"]["sequence"] = rand() % 10000;  // 随机序列号
            return j;
        }
        
        // check if it is heartbeat_message
        static bool is_heartbeat(const json& j) {
            return j.value("type", "") == MSG_TYPE_HEARTBEAT;
        }

        static json make_pose_message(const std::string& topic_name,
                                       const geometry_msgs::PoseStamped& msg) {
            // msg is not ptr;
            json j;
            j["type"] = MSG_TYPE_POSE;
            j["topic"] = topic_name;
            // no user
            j["timestamp"] = ros::Time::now().toSec(); // use the current ROS time
            // header
            j["data"]["header"]["seq"] = msg.header.seq;
            j["data"]["header"]["stamp"] = msg.header.stamp.toSec();
            j["data"]["header"]["frame_id"] = msg.header.frame_id;
            
            auto& p = msg.pose.position;
            auto& o = msg.pose.orientation;

            j["data"]["pose"]["position"] = {{"x", p.x}, {"y", p.y}, {"z", p.z}};           // 修复：改为对象格式
            j["data"]["pose"]["orientation"] = {{"x", o.x}, {"y", o.y}, {"z", o.z}, {"w", o.w}}; // 修复：改为对象格式

            return j;
        }

        static json make_string_message(const std::string& topic_name,
                                        const std_msgs::String& msg) {
            json j;
            j["type"] = MSG_TYPE_STRING;
            j["topic"] = topic_name;
            j["timestamp"] = ros::Time::now().toSec(); 
            j["data"]["message"] = msg.data;
            return j;
        }

        static void parse_received(const json& j, 
                                    std::string& topic_name,
                                    std::string& msg_type,
                                    json& data,
                                    double& timestamp) {
            msg_type = j.value("type", "unknown");
            topic_name = j.value("topic", "unknown");
            timestamp = j.value("timestamp", 0.0);
            data = j.contains("data") ? j["data"] : json::object();
        }
};