#pragma once
#include <functional>
#include "auto_serial_bridge/serial_controller.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "protocol.h"

namespace auto_serial_bridge {
namespace generated {

template <typename T> void register_subscriber(SerialController* node, const std::string& topic, PacketID id);

inline void register_all(SerialController* node) {
    // Heartbeat (ROS -> MCU)
    node->add_subscription(node->create_subscription<std_msgs::msg::UInt32>(
        "/task/heartbeat", 10,
        [node](const std_msgs::msg::UInt32::SharedPtr msg) {
            Packet_Heartbeat pkt;
            pkt.count = msg->data;
            node->send_packet(PACKET_ID_HEARTBEAT, pkt);
        }));

    // Handshake (ROS -> MCU)
    node->add_subscription(node->create_subscription<std_msgs::msg::UInt32>(
        "/task/handshake", 10,
        [node](const std_msgs::msg::UInt32::SharedPtr msg, const rclcpp::MessageInfo& msg_info) {
            if (node->should_skip_loopback(PACKET_ID_HANDSHAKE, msg_info)) {
                return;
            }
            Packet_Handshake pkt;
            pkt.protocol_hash = msg->data;
            node->send_packet(PACKET_ID_HANDSHAKE, pkt);
        }));

    // CmdVel (ROS -> MCU)
    node->add_subscription(node->create_subscription<geometry_msgs::msg::Twist>(
        "/serial/chassis_cmd", 10,
        [node](const geometry_msgs::msg::Twist::SharedPtr msg) {
            Packet_CmdVel pkt;
            pkt.turn_angle = msg->angular.z;
            pkt.turn_wheels = msg->linear.z;
            pkt.forward_dist = msg->linear.x;
            pkt.drive_wheels = msg->linear.y;
            pkt.pickup_action = msg->angular.x;
            pkt.search_rotate = msg->angular.y;
            node->send_packet(PACKET_ID_CMDVEL, pkt);
        }));

    // SlopeInfo (ROS -> MCU)
    node->add_subscription(node->create_subscription<geometry_msgs::msg::Vector3>(
        "/serial/slope_info", 10,
        [node](const geometry_msgs::msg::Vector3::SharedPtr msg) {
            Packet_SlopeInfo pkt;
            pkt.angle_deg = msg->x;
            pkt.horiz_dist = msg->y;
            pkt.height_diff = msg->z;
            node->send_packet(PACKET_ID_SLOPEINFO, pkt);
        }));

    // MeilinCmd (ROS -> MCU)
    node->add_subscription(node->create_subscription<geometry_msgs::msg::Twist>(
        "/serial/meilin_cmd", 10,
        [node](const geometry_msgs::msg::Twist::SharedPtr msg) {
            Packet_MeilinCmd pkt;
            pkt.next_block = msg->linear.x;
            pkt.climb_mode = msg->linear.y;
            pkt.slope_angle = msg->linear.z;
            pkt.block_height = msg->angular.x;
            pkt.detour = msg->angular.y;
            node->send_packet(PACKET_ID_MEILINCMD, pkt);
        }));

}

struct ProtocolPublishers {
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_Handshake;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_GripperStatus;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_AssemblyStatus;

    void init(SerialController* node) {
        pub_Handshake = node->create_publisher<std_msgs::msg::UInt32>("/task/handshake", 10);
        node->register_loopback_publisher(PACKET_ID_HANDSHAKE, pub_Handshake);
        pub_GripperStatus = node->create_publisher<std_msgs::msg::UInt8>("/feedback/gripper", 10);
        pub_AssemblyStatus = node->create_publisher<std_msgs::msg::UInt8>("/feedback/assembly", 10);
    }
};

inline void dispatch_packet(ProtocolPublishers& pubs, uint8_t id, const std::vector<uint8_t>& data) {
    switch(id) {
        case PACKET_ID_HANDSHAKE: {
            if (data.size() != sizeof(Packet_Handshake)) break;
            const Packet_Handshake* pkt = reinterpret_cast<const Packet_Handshake*>(data.data());
            auto msg = std_msgs::msg::UInt32();
            msg.data = pkt->protocol_hash;
            if (pubs.pub_Handshake) {
                pubs.pub_Handshake->publish(msg);
            }
            break;
        }
        case PACKET_ID_GRIPPERSTATUS: {
            if (data.size() != sizeof(Packet_GripperStatus)) break;
            const Packet_GripperStatus* pkt = reinterpret_cast<const Packet_GripperStatus*>(data.data());
            auto msg = std_msgs::msg::UInt8();
            msg.data = pkt->status;
            if (pubs.pub_GripperStatus) {
                pubs.pub_GripperStatus->publish(msg);
            }
            break;
        }
        case PACKET_ID_ASSEMBLYSTATUS: {
            if (data.size() != sizeof(Packet_AssemblyStatus)) break;
            const Packet_AssemblyStatus* pkt = reinterpret_cast<const Packet_AssemblyStatus*>(data.data());
            auto msg = std_msgs::msg::UInt8();
            msg.data = pkt->status;
            if (pubs.pub_AssemblyStatus) {
                pubs.pub_AssemblyStatus->publish(msg);
            }
            break;
        }
    }
}
}
}
