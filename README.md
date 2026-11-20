# AKM 底盘控制框架

面向 ROS 2 Humble 的阿克曼底盘控制包，包含 SocketCAN 通信、协议编解码、
速度指令转换、状态发布和无硬件仿真模式。定位与里程计由独立 SLAM 模块负责。

## 构建

```bash
source /opt/ros/humble/setup.zsh
colcon build
source install/setup.zsh
```

以上命令适用于本机的 zsh；如果使用 Bash 终端，再将后缀改为 `.bash`。

## 运行

直接运行构建目录中的二进制，默认配置决定实车或仿真模式：

```bash
./bin/fox_control
```

需要临时切换到仿真模式时：

```bash
./bin/fox_control --ros-args -p simulated_robot:=true
```

节点订阅 `/cmd_vel`，发布 `/akm_status`，不发布里程计和定位 TF。
程序通过 C++ 和 yaml-cpp 自动读取配置，无需传 `--params-file`。
配置文件绝对路径在 `src/akm_base_node.cpp` 的 `kConfigPath` 中指定。

* Then you can send command to the robot

  ```
    $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0" 

    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.2}}"
    ```
## 安全行为

- 线速度和转角按配置上限约束。
- 超过 `command_timeout` 未收到速度指令时自动发送零速命令。
- 节点退出前发送零速命令。
- CAN 执行器编号在写入状态数组前进行边界检查。
