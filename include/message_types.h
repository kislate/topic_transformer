#pragma once
#include "json.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

using json = nlohmann::json;

const std::string MSG_TYPE_STRING = "string";
const std::string MSG_TYPE_POSE = "pose_stamped";

class MessageTypes {
    public:
        static json make_pose_message(const std::string& topic_name,
                                       const geometry_msgs::PoseStamped& msg) {
            json j;
            j["type"] = MSG_TYPE_POSE;
            j["topic"] = topic_name;
            j["timestamp"] = ros::Time::now().toSec();
            j["data"]["stamp"] = msg->header.stamp.toSec();
            j["data"]["frame_id"] = msg->header.frame_id;
            
            auto& p = msg->pose.position;
            auto& o = msg->pose.orientation;

            j["data"]["position"]["x"] = {p.x, p.y, p.z};
            j["data"]["orientation"]["x"] = {o.x, o.y, o.z, o.w};

            return j;
        }

        static json make_string_message(const std::string& topic_name,
                                        const std_msgs::String& msg) {
            json j;
            j["type"] = MSG_TYPE_STRING;
            j["topic"] = topic_name;
            j["timestamp"] = ros::Time::now().toSec();
            j["data"]["message"] = msg->data;
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