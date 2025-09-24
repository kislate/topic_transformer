# Topic Transformer

一个ROS包，用于通过UDP网络传输ROS话题，实现跨网络的话题通信。

## 功能特性

- 通过UDP发送ROS话题到远程主机
- 接收UDP消息并发布为ROS话题
- 支持 `geometry_msgs/PoseStamped` 和 `std_msgs/String` 消息类型
- 心跳机制确保连接状态
- YAML配置文件，便于扩展和维护

## 配置文件

编辑 `config/transformer_config.yaml` 来配置网络和话题设置：

```yaml
# 网络配置
network:
  local_ip: "0.0.0.0"           # 本地IP，0.0.0.0表示绑定所有接口
  local_port: 8001              # 本地监听端口
  dest_ip: "192.168.1.100"      # 目标主机IP
  dest_port: 8002               # 目标主机端口

# 心跳配置
heartbeat:
  enabled: true
  interval_ms: 3000             # 心跳间隔（毫秒）

# 话题转发配置 - ROS话题 -> UDP
publish_topics:
  - topic_name: "/robot/pose"
    topic_type: "pose_stamped"
  - topic_name: "/robot/status"
    topic_type: "string"

# 话题接收配置 - UDP -> ROS话题  
subscribe_topics:
  - topic_name: "/remote/cmd"
    topic_type: "string"
  - topic_name: "/remote/goal"
    topic_type: "pose_stamped"
```

## 使用方法

### 1. 编译包

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 修改配置

根据您的网络环境修改 `config/transformer_config.yaml`。

### 3. 启动节点

```bash
roslaunch topic_transformer transformer.launch
```

### 4. 测试

发布测试消息：

```bash
# 发布字符串消息
rostopic pub /robot/status std_msgs/String "data: 'Hello from robot'"

# 发布位置消息
rostopic pub /robot/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base_link'
pose:
  position:
    x: 1.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

监听接收到的消息：

```bash
# 监听从UDP接收的消息
rostopic echo /remote/cmd
rostopic echo /remote/goal
```

## 工作原理

1. **ROS话题 -> UDP**: 节点订阅配置中的ROS话题，将接收到的消息转换为JSON格式通过UDP发送
2. **UDP -> ROS话题**: 节点监听UDP端口，将接收到的JSON消息解析后发布为ROS话题
3. **心跳机制**: 定期发送心跳包检测网络连接状态

## 支持的消息类型

- `geometry_msgs/PoseStamped` - 位置姿态信息
- `std_msgs/String` - 字符串消息

可通过修改 `message_types.h` 和相应的处理代码来支持更多消息类型。

一个用于在不同 ROS 节点之间通过 UDP 转发消息的工具。支持将 ROS 消息序列化为 JSON 格式进行网络传输。

## 功能特性

- 支持多种 ROS 消息类型（目前支持 `geometry_msgs/PoseStamped` 和 `std_msgs/String`）
- UDP 网络通信，支持跨机器的消息转发
- JSON 序列化/反序列化
- 异步消息接收与回调处理
- 线程安全的启停控制

## 系统要求

### Linux (推荐)
- Ubuntu 18.04+ 或其他 Linux 发行版
- ROS Melodic/Noetic
- C++11 及以上编译器

### Windows (有限支持)
由于使用了 POSIX socket API，建议在 Windows 上使用以下方式：
- WSL (Windows Subsystem for Linux)
- Docker 容器
- 或修改代码使用 Winsock API

## 依赖项

```bash
# ROS 核心包
sudo apt install ros-<distro>-geometry-msgs ros-<distro>-std-msgs

# nlohmann/json (已包含在 include/json.hpp)
```

## 编译配置

### CMakeLists.txt 示例
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(topic_transformer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/udp_communicator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)
```

### package.xml 示例
```xml
<package format="2">
  <name>topic_transformer</name>
  <version>1.0.0</version>
  <description>UDP-based ROS message forwarder</description>
  
  <buildtool_depend>catkin</buildtool_depend>
  
  <depend>roscpp</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
</package>
```

## 使用方法

### 1. 基本初始化

```cpp
#include "udp_communicator.h"

// 创建 UDP 通信器
// 参数：本地IP, 本地端口, 目标IP, 目标端口
UDPCommunicator comm("192.168.1.100", 8888, "192.168.1.101", 8889);
```

### 2. 设置消息接收回调

```cpp
// 设置回调函数处理接收到的消息
comm.set_message_callback([](const std::string& topic, const json& data, double timestamp) {
    std::cout << "收到消息 - 话题: " << topic 
              << ", 时间戳: " << timestamp << std::endl;
    
    // 根据消息类型处理
    std::string msg_type, topic_name;
    json msg_data;
    double ts;
    
    // 修复：使用正确的解析方法
    MessageTypes::parse_received(data, topic_name, msg_type, msg_data, ts);
    
    if (msg_type == MSG_TYPE_POSE) {
        // 处理位置消息 - 修复：访问正确的数据结构
        auto pos = msg_data["position"];
        ROS_INFO("位置: x=%.2f, y=%.2f, z=%.2f", 
                 pos["x"].get<double>(),
                 pos["y"].get<double>(), 
                 pos["z"].get<double>());
    }
    else if (msg_type == MSG_TYPE_STRING) {
        // 处理字符串消息
        ROS_INFO("消息内容: %s", msg_data["message"].get<std::string>().c_str());
    }
});
```

### 3. 启动和停止通信

```cpp
// 启动 UDP 通信
if (comm.start()) {
    ROS_INFO("UDP 通信启动成功");
} else {
    ROS_ERROR("UDP 通信启动失败");
    return -1;
}

// 程序结束时停止
comm.stop();
```

### 4. 发送消息

```cpp
// 发送 PoseStamped 消息
geometry_msgs::PoseStamped pose_msg;
pose_msg.header.frame_id = "base_link";
pose_msg.header.stamp = ros::Time::now();
pose_msg.pose.position.x = 1.0;
pose_msg.pose.position.y = 2.0;
pose_msg.pose.position.z = 3.0;

json pose_json = MessageTypes::make_pose_message("/robot_pose", pose_msg);
comm.send_json(pose_json);

// 发送 String 消息
std_msgs::String str_msg;
str_msg.data = "Hello, ROS!";

json str_json = MessageTypes::make_string_message("/status", str_msg);
comm.send_json(str_json);
```

## 完整示例

### 发送端节点
```cpp
#include <ros/ros.h>
#include "udp_communicator.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_sender");
    ros::NodeHandle nh;
    
    UDPCommunicator comm("127.0.0.1", 8888, "127.0.0.1", 8889);
    
    if (!comm.start()) {
        ROS_ERROR("无法启动 UDP 通信");
        return -1;
    }
    
    ros::Rate rate(10); // 10Hz
    
    while (ros::ok()) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "base_link";
        pose.pose.position.x = sin(ros::Time::now().toSec());
        
        json msg = MessageTypes::make_pose_message("/robot_pose", pose);
        comm.send_json(msg);
        
        rate.sleep();
    }
    
    comm.stop();
    return 0;
}
```

### 接收端节点
```cpp
#include <ros/ros.h>
#include "udp_communicator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_receiver");
    ros::NodeHandle nh;
    
    UDPCommunicator comm("127.0.0.1", 8889, "127.0.0.1", 8888);
    
    // 设置接收回调
    comm.set_message_callback([](const std::string& topic, const json& data, double ts) {
        ROS_INFO("接收到话题 %s 的消息，时间戳: %.3f", topic.c_str(), ts);
    });
    
    if (!comm.start()) {
        ROS_ERROR("无法启动 UDP 通信");
        return -1;
    }
    
    ros::spin();
    
    comm.stop();
    return 0;
}
```

## JSON 消息格式

### PoseStamped 消息格式
```json
{
  "type": "pose_stamped",
  "topic": "/robot_pose",
  "timestamp": 1634567890.123,
  "data": {
    "stamp": 1634567890.120,
    "frame_id": "base_link",
    "position": {
      "x": 1.0,
      "y": 2.0,
      "z": 3.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  }
}
```

### String 消息格式
```json
{
  "type": "string",
  "topic": "/status",
  "timestamp": 1634567890.123,
  "data": {
    "message": "Hello, ROS!"
  }
}
```

## 注意事项

1. **网络配置**：确保防火墙允许指定端口的 UDP 通信
2. **消息大小**：UDP 有最大传输单元限制，大消息可能需要分片
3. **可靠性**：UDP 不保证消息到达，关键应用请考虑添加确认机制
4. **线程安全**：接收回调在独立线程中执行，注意线程安全

## 扩展支持

要添加新的消息类型支持，在 `MessageTypes` 类中添加相应的 `make_*_message` 方法：

```cpp
static json make_twist_message(const std::string& topic_name,
                              const geometry_msgs::Twist& msg) {
    json j;
    j["type"] = "twist";
    j["topic"] = topic_name;
    j["timestamp"] = ros::Time::now().toSec();
    j["data"]["linear"]["x"] = msg.linear.x;
    j["data"]["linear"]["y"] = msg.linear.y;
    j["data"]["linear"]["z"] = msg.linear.z;
    j["data"]["angular"]["x"] = msg.angular.x;
    j["data"]["angular"]["y"] = msg.angular.y;
    j["data"]["angular"]["z"] = msg.angular.z;
    return j;
}
```

## 故障排除

- **编译错误**：检查 ROS 环境是否正确 source，依赖包是否安装
- **连接失败**：检查 IP 地址和端口配置，确认网络连通性
- **消息丢失**：考虑网络拥塞或缓冲区溢出，可适当降低发送频率