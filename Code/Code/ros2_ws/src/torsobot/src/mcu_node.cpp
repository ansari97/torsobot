#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <cstring>

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

    // initialize pigpio
    pi_gpio = gpioInitialise();
    if (pi_gpio < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error initializing pigpio");
      // cerr << "Error initializing pigpio" << endl;
      usleep(100);
      // return -1;
    };
    RCLCPP_INFO(this->get_logger(), "Initialized pigpio");
    // cout << "Initialized pigpio" << endl;

    // open I2C channel
    i2c_handle = i2cOpen(I2C_BUS, PICO_SLAVE_ADDRESS, 0);
    if (i2c_handle < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device: %d", i2c_handle);
      // cerr << "Failed to open I2C device: " << i2c_handle << endl;
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
    int write_request = i2cWriteByte(i2c_handle, cmd);
    if (write_request != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "I2C write failed! %d", write_request);
      // cerr << "I2C write failed! " << write_request << endl;
      return write_request;
    }

    // Rrequest sensor data from slave
    int read_result = i2cReadDevice(i2c_handle, read_buff, data_len);

    if (read_result != data_len)
    {
      RCLCPP_ERROR(this->get_logger(), "I2C read failed! %d", read_result);
      // cerr << "I2C read failed! " << read_result << endl;
      return read_result;
    }
    else
    {
      memcpy(sensor_val_addr, read_buff, data_len);
      // cout << "Read data: " << *sensor_val_addr << endl;
    }

    return 1;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mcuNode>());
  rclcpp::shutdown();
  return 0;
}
