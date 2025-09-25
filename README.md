# Topic Transformer

一个ROS包，用于通过UDP网络传输ROS话题，实现跨网络的话题通信。支持双向通信：本地话题通过UDP发送到远程，远程消息通过UDP接收并发布到本地。

## 功能特性

- **双向话题传输**：支持本地→远程和远程→本地的话题转发
- **多消息类型**：支持 `geometry_msgs/PoseStamped` 和 `std_msgs/String` 消息类型
- **心跳机制**：定期发送心跳包检测网络连接状态
- **灵活配置**：支持多个话题订阅，通过YAML配置文件管理
- **话题前缀**：自动为接收到的远程话题添加前缀标识

## 配置文件

编辑 `config/transformer_config.yaml` 来配置网络和话题设置：

```yaml
# 飞机编号
drone_id: 0
topic_prefix: "drone_1"      # 来自远程的话题前缀

# 网络配置
local_ip: "0.0.0.0"          # 本地IP，0.0.0.0表示绑定所有接口
local_port: 46666            # 本地监听端口
dest_ip: "192.168.1.100"     # 目标主机IP
dest_port: 46667             # 目标主机端口

# 心跳配置
heartbeat_enabled: true
heartbeat_interval_ms: 10000  # 心跳间隔（毫秒）

# 发送的话题配置 - 本地话题 -> UDP
sub_topic_num: 2              # 要发送的话题数量
sub_topic_1_name: "mavros/local_position/pose"
sub_topic_1_type: "pose_stamped"
sub_topic_2_name: "status"
sub_topic_2_type: "string"
```

## 使用方法

### 1. 编译包

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 修改配置

根据您的网络环境修改 `config/transformer_config.yaml`：

- 设置正确的网络参数（IP地址和端口）
- 配置要发送的本地话题
- 设置话题前缀用于区分远程消息

### 3. 启动节点

```bash
roslaunch topic_transformer transformer.launch
```

或者使用自定义参数：

```bash
rosrun topic_transformer topic_transformer_node \
  _local_ip:="0.0.0.0" \
  _local_port:=46666 \
  _dest_ip:="192.168.1.100" \
  _dest_port:=46667 \
  _sub_topic_num:=1 \
  _sub_topic_1_name:="mavros/local_position/pose" \
  _sub_topic_1_type:="pose_stamped"
```

### 4. 测试发送功能

发布本地话题（会通过UDP发送到远程）：

```bash
# 发布位置消息（如果配置了pose类型话题）
rostopic pub /mavros/local_position/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'base_link'
pose:
  position: {x: 1.0, y: 2.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

# 发布字符串消息（如果配置了string类型话题）
rostopic pub /status std_msgs/String "data: 'Hello from drone'"
```

### 5. 监听接收功能

监听从UDP接收的远程消息（会自动添加前缀）：

```bash
# 监听远程发来的消息（假设topic_prefix为"drone_1"）
rostopic echo /drone_1/mavros/local_position/pose
rostopic echo /drone_1/status

# 查看所有可用话题
rostopic list | grep drone_1
```

## 工作原理

### 双向通信架构

```
本地ROS节点 → 订阅器 → JSON转换 → UDP发送 → 远程主机
                                                    ↓
远程ROS节点 ← 发布器 ← JSON解析 ← UDP接收 ← 远程主机
```

### 消息流程

1. **本地→远程（发送）**：
   - 订阅配置中的本地ROS话题
   - 将ROS消息转换为JSON格式
   - 通过UDP发送到目标主机

2. **远程→本地（接收）**：
   - 监听UDP端口接收远程消息
   - 解析JSON消息并转换为ROS消息
   - 添加话题前缀后发布到本地ROS网络

3. **心跳机制**：
   - 定期发送心跳包检测连接状态
   - 监控对端是否在线

## 支持的消息类型

- **geometry_msgs/PoseStamped** - 位置姿态信息
- **std_msgs/String** - 字符串消息

要支持更多消息类型，需要：
1. 在 `message_types.h` 中添加转换函数
2. 在主程序中添加对应的处理逻辑

## 多机器人配置示例

### 机器人A配置（drone_0）
```yaml
drone_id: 0
topic_prefix: "drone_1"
local_port: 46666
dest_ip: "192.168.1.102"
dest_port: 46667
sub_topic_1_name: "mavros/local_position/pose"
```

### 机器人B配置（drone_1）
```yaml
drone_id: 1
topic_prefix: "drone_0"
local_port: 46667
dest_ip: "192.168.1.101"
dest_port: 46666
sub_topic_1_name: "mavros/local_position/pose"
```

这样配置后：
- 机器人A会接收到 `/drone_1/mavros/local_position/pose`
- 机器人B会接收到 `/drone_0/mavros/local_position/pose`

## 故障排除

### 常见问题

1. **UDP端口被占用**：
   ```bash
   netstat -tulpn | grep :46666
   ```

2. **网络连接问题**：
   ```bash
   ping 192.168.1.100
   telnet 192.168.1.100 46667
   ```

3. **消息未发送**：
   - 检查话题名称是否正确
   - 确认消息类型匹配
   - 查看ROS日志输出

### 调试命令

```bash
# 查看节点状态
rosnode info /topic_transformer_node

# 监控网络流量
sudo tcpdump -i any port 46666

# 查看ROS话题
rostopic list
rostopic info /your_topic_name
```

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