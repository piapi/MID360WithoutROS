## 这是一个使用官方的ROS版本的驱动修改得到不使用ROS的LIVOX-MID360的驱动

1. 需要先安装MID360的SDK（按照官方的安装教程即可）
2. 在LivoxMid360.cpp中  `std::string m_IP = "192.168.192.46"`设置雷达的IP地址
3. 在`MID360_config.json` 文件中，修改      `"cmd_data_ip" : "192.168.192.50", "push_msg_ip"，  "point_data_ip"，  "imu_data_ip"` 为你的本机的IP地址
4. 在pub_Handler的`uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size)`，通过`timestamp_type` 来设置获取道德数据的时间戳类型，本工程设置为获取硬件时间
5. 然后链接上雷达编译运行即可



