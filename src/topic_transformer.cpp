#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include "udp_communicator.h"

UDPCommunicator* udp_comm = nullptr;

// send pose messages as JSON powered by UDP
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (udp_comm) {
        json j = MessageTypes::make_pose_message("pose_topic", *msg);
        udp_comm->send_json(j);
    }
}

void stringCallback(const std_msgs::String::ConstPtr& msg) {
    if (udp_comm) {
        json j = MessageTypes::make_string_message("string_topic", *msg);
        udp_comm->send_json(j);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "topic_transformer");
    ros::NodeHandle nh;

    // Initialize UDP Communicator
    udp_comm = new UDPCommunicator("0.0.0.0", 114514, "100.67.3.30", 114514);

    udp_comm->set_message_callback(
        [](const std::string& topic, const json& data, double timestamp) {
            ROS_INFO("Received on topic: %s at time: %f", topic.c_str(), timestamp);
            // Handle different message types based on 'data'
            if (data.contains("message")) {
                ROS_INFO("String message: %s", data["message"].get<std::string>().c_str());
            } else if (data.contains("position") && data.contains("orientation")) {
                auto pos = data["position"]["x"];
                auto ori = data["orientation"]["x"];
                ROS_INFO("Pose - Position: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
                         pos[0].get<double>(), pos[1].get<double>(), pos[2].get<double>(),
                         ori[0].get<double>(), ori[1].get<double>(), ori[2].get<double>(), ori[3].get<double>());
            } else {
                ROS_WARN("Unknown message format.");
            }
        }
    );

    if (!udp_comm->start()) {
        ROS_ERROR("Failed to start UDP Communicator.");
        delete udp_comm;
        return -1;
    }

    ros::Subscriber pose_sub = nh.subscribe("pose_topic", 10, poseCallback);
    ros::Subscriber string_sub = nh.subscribe("string_topic", 10, stringCallback);

    ros::spin();

    delete udp_comm;
    return 0;
}