#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "../include/imu_im948/im948_CMD.h" 

#define COM_PORT "/dev/ttyUSB0"

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imuMsg_pub;

class IMUNode : public rclcpp::Node {
public:
    IMUNode() : Node("imu_node") {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imuData_raw", 20);
        imu_command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "imuSendCmd", 10, std::bind(&IMUNode::imuSendCmdCallback, this, std::placeholders::_1));

        imuMsg_pub = imu_publisher_;  // **确保全局变量初始化**

        try {
            serial_.setPort(COM_PORT);
            serial_.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", COM_PORT);
            rclcpp::shutdown();
        }

        if (!serial_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "串口打开失败");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "串口初始化成功");
        }

        try {
            Cmd_12(5, 255, 0, 0, 2, 60, 1, 3, 5, 0x0026);
            Cmd_03();
            Cmd_19();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IMU 初始化失败: %s", e.what());
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5), std::bind(&IMUNode::readIMUData, this));
    }

private:
    serial::Serial serial_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_command_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    void imuSendCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "发送串口命令: %s", msg->data.c_str());
        serial_.write(msg->data);
    }

    void readIMUData() {
        if (!serial_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "串口已关闭，无法读取数据");
            return;
        }
        if (serial_.available()) {
            unsigned char buffer[512];
            size_t bytes_to_read = std::min(static_cast<size_t>(serial_.available()), sizeof(buffer));
            size_t bytes_read = serial_.read(buffer, bytes_to_read);

            for (size_t i = 0; i < bytes_read; i++) {
                Cmd_GetPkt(buffer[i]);
            }
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}
