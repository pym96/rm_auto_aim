# RV: rm_auto_aim, 原华南师范大学pioneer开源的一套视觉自瞄方案，因其high robustness还有效果不错的解算方式，故写成文档以供MA视觉组（算法）参考学习（建议参考代码的同时结合该文档使用），虽然君佬已经放手，但仍然感谢他以及华南师范战队的开源付出......

# 简介：

**文档携程方式采用基于IMU的视觉方案层层递进**

# 目录：

## 1. 串口模块

### 代码架构

```c++
/**
rm_serial_driver
	-- config： 串口configuration file, 比如串口的逻辑名称，波特率等等 
		-- serial_driver.yaml
	-- docs： RV标， 宣示主权是吧， 淦
	-- include： 结点类或者工具类的头文件
		-- crc.hpp
		-- packer.hpp
		-- rm_serial_driver.hpp
	-- launch： launch, ros语法，用于同时启动多个节点
	-- src： 结点类或工具类的实现
*/
```

## 代码解析：

```C++
// 注：每次的代码解析都将从可执行的Node说起， 对应于该 rm_serial_driver package, 可执行的node 是src/ 下的rm_serial_driver.cpp文件


/**
	包含ROS的头文件
*/
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// C++ system 头文件
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>


// 以下为自定义头文件
#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"


namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();
    
  ********************************getParams() begin***********************************
//  getParams() 解析：
  
  void RMSerialDriver::getParams(){
  
  // 首先是定义三个别名， 这三个类型出自ros2 driver pakcage的包， 如果还没有下载， 请输入以下命令下载：
  // sudo apt install ros-humble-serial-driver
  // serial driver包的文档内容我均已上传到了MA_training_advanced.md, 如果不了解相关内容，请参考文档，地址是：https://github.com/pym96/MA_training
  // 以下文档内容我均假设读者熟悉相关模块的基础知识而展开
      
  using FlowControl = drivers::serial_driver::FlowControl; 
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{}; // 波特率的设置， 单位是 码元/秒
  auto fc = FlowControl::NONE; // Flowcontrol 表示串行通信中的数据流控制方式，这里采用NONE, 即表示没有数据流的控制， 在这种情况下， 数据发送方和接收方之间没有流控制信号的交互， 数据按照原始速率发送和接受。
  auto pt = Parity::NONE;  // 一个每局类型，表示串行通信(Serial communication)中的奇偶校验（Parity) 位， 但我们采用的是CRC校验方式，故此设为空
  auto sb = StopBits::ONE;  // 停止位， 首先要明确， 在串行通信中， 每个数据字节都由数据位，校验位（可以是CRC或者奇偶校验）和停止位组成。 停止位用来标识一个数据字节的结束。在每个数据字节的后边， 发送方会插入指定数量的停止位， 接收方在接收数据时，根据停止位的位置来识别数据帧的边界。这里采用一个字节当作停止位。

      
  // 初始波特率
   try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }
  
      
  // 初始化flow control 的方式
   try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }
      
   
  // 初始化校验方式
   try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }
   
   // 初始化停止位信息
   try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }
       device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

  *********************************getParams() end***********************************
      
      
   // TF broadcaster: 此部分建议依然参考 MA_vision_advance.md tf 讲解部分
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    
 *****************************************TF broadcaster begin*******************************************
// Broadcast tf from odom to gimbal_link, gimbal_link： 你可以将其想象成底盘，但是用URDF 描述， 此部分文档依然在MA_Vision_advance.md 中， 可以去参考
// 上边两个变量均定义在rm_serial_driver package 中， 文件名是： rm_serial_driver.hpp
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // 智能unique指针， 不同于shared_ptr, unique_ptr的互斥性更强
  
******************************************TF broadcaster begin end**************************************
    
    
// Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    
    
 *****************************************Create Publisher begin*******************************************
// For debug usage
     
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_; // 定义发布者发送时延
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // visualization_msgs, 用于相关组件在rviz中的可视化， 不了解请去看 MA_vision_advanced.md

 *****************************************Create Publisher end*******************************************
     
     
// Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
    
 ***************************************** Detect parameter client begin*******************************************
	关于 ROS 的进程控制以及 service client 内容请参考 MA_vision_advanced.md 文档     
 ***************************************** Detect parameter client end*********************************************
     
// Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

***************************************** Tracker reset service client begin*******************************************
   参考内容 MA_vision_advanced.md ROS 通用知识， service, 参考库文档部分
***************************************** Tracker reset service client begin*******************************************
    
// 打开串口
 try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
    
***************************************** 打开串口 begin*******************************************
   #include <serial_driver/serial_driver.hpp>
   #include <memory>
    // std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver;
    // std::unique_prt<drivers::serial_driver::SerialPortConfig> device_config;
    serial_driver->init_port(device_name, *device_config_);
    
    if (!serial_driver->port()->is_open()){
       serial_driver->port()->is_open(); // 打开串口
       receive_thread = std::thread(&RMSeialDricer::receiveData, this);
    }
    
***************************************** 打开串口 end*******************************************
```

