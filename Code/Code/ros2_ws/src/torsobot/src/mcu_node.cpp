#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
// using namespace std;

// pico slave address
#define PICO_SLAVE_ADDRESS 0X30

// I2C_bus 1 for PI
const int I2C_BUS = 1;

int pi_gpio, i2c_handle;

// Buffers to read data and store vals
const int data_len = sizeof(float);
char read_buff[data_len];
float IMU_val;

// I2C write cmd for IMU data
const int IMU_CMD = 0x01;

// function to get sensor value for some cmd
int GetSensorValue(int, float *);

class mcuNode : public rclcpp::Node
{
public:
  mcuNode()
      : Node("mcu_node"), count_(0)
  {
    // Create publisher for "torso_angle" topic
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("torso_angle", 10);

    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", I2C_BUS);
    // initialize i2c
    if ((i2c_handle = open(filename, O_RDWR)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error opening i2c");
      // return 1;
    }
    RCLCPP_INFO(this->get_logger(), "Opened i2c");
    // cout << "Initialized pigpio" << endl;

    // open I2C channel
    // Specify slave address
    if (ioctl(i2c_handle, I2C_SLAVE, PICO_SLAVE_ADDRESS) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device: %d", i2c_handle);
      close(i2c_handle);
      // return 1;
    }
    RCLCPP_INFO(this->get_logger(), "Successfully opened I2C device: %d", i2c_handle);
    // cout << "Successfully opened I2C: " << i2c_handle << endl;

    auto timer_callback =
        [this]() -> void
    {
      // Read / request sensor data from pico
      int get_sensor_val_result1 = GetSensorValue(IMU_CMD, &IMU_val);
      if (get_sensor_val_result1 <= 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Error reading sensor value");
        // return 0;
      }
      // cout << "Sensor 1 data: " << IMU_val << endl;

      auto message = std_msgs::msg::Float32();
      message.data = IMU_val;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
      this->publisher_->publish(message);
    };

    // Timer for 100ms (10Hz)
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  size_t count_;

  // Get sensor data over I2C
  int GetSensorValue(int cmd, float *sensor_val_addr)
  {
    // Clear the read buffer
    memset(read_buff, 0, data_len);

    // Write command to slave
    uint8_t commandByte = static_cast<uint8_t>(cmd);
    if (write(i2c_handle, &commandByte, 1) != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "I2C write failed! ");
      return -1; // Indicate error
    }

    // Request sensor data from slave
    if (read(i2c_handle, read_buff, data_len) != data_len)
    {
      RCLCPP_ERROR(this->get_logger(), "I2C read failed! ");
      return -2; // Indicate error
    }
    else
    {
      memcpy(sensor_val_addr, read_buff, data_len);
    }

    return 1; // Success
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mcuNode>());
  rclcpp::shutdown();
  return 0;
}
