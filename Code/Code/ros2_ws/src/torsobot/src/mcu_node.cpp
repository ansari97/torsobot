#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
// #include <unistd.h>
#include <cstring>
#include <cstdint>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "torsobot_interfaces/msg/data.hpp"

using namespace std::chrono_literals;

// pico slave address
#define PICO_SLAVE_ADDRESS 0X30

// I2C_bus 1 for PI
const int I2C_BUS = 1;

int i2c_handle;

// Buffers to read data and store vals
const int data_len = sizeof(float);
char read_buff[data_len];
float IMU_val;

// I2C write cmd for IMU data
const int IMU_CMD = 0x01;

class mcuNode : public rclcpp::Node
{
public:
  mcuNode() : Node("mcu_node")
  {
    // Create publisher for "torso_angle" topic
    publisher_ = this->create_publisher<torsobot_interfaces::msg::Data>("pico_data", 10);

    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", I2C_BUS);

    // initialize i2c
    if ((i2c_handle = open(filename, O_RDWR)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error opening i2c, error: %d", i2c_handle);
      // return 1;
    }
    RCLCPP_INFO(this->get_logger(), "Opened i2c");

    // open I2C channel
    // Specify slave address
    if (ioctl(i2c_handle, I2C_SLAVE, PICO_SLAVE_ADDRESS) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device!");
      close(i2c_handle);

      RCLCPP_FATAL(this->get_logger(), "Failed to open I2C device! Exiting! :(");
      rclcpp::shutdown();
      exit(EXIT_FAILURE); // Terminate the program
      // return 1;
    }
    RCLCPP_INFO(this->get_logger(), "Successfully opened I2C device!");

    // Timer for 10ms (100Hz)
    timer_ = this->create_wall_timer(10ms, std::bind(&mcuNode::timer_callback, this)); // Use std::bind
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<torsobot_interfaces::msg::Data>::SharedPtr publisher_;

  // timer callback for getting data on i2c
  void timer_callback(void)
  {
    // Read / request sensor data from pico
    int get_sensor_val_result1 = GetSensorValue(IMU_CMD, &IMU_val);
    if (get_sensor_val_result1 > 0)
    {
      auto message = torsobot_interfaces::msg::Data();
      message.torso_pitch = IMU_val;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.torso_pitch);
      this->publisher_->publish(message);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error reading sensor value");
    }
  };

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
