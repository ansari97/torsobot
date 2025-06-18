#include "rclcpp/rclcpp.hpp"
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
#include <functional>
#include <gpiod.hpp> //for GPIO control

#include "rclcpp/rclcpp.hpp"
#include "torsobot_interfaces/msg/torsobot_data.hpp"

using namespace std::chrono_literals;

// Configuration for the GPIO pin
const std::string GPIO_CHIP_NAME = "gpiochip0"; // Verify with 'ls /dev/gpiochip*' on your Pi5
const unsigned int RPI_GPIO_FOR_PICO_RUN = 14;  // GPIO14

// pico slave address
#define PICO_SLAVE_ADDRESS 0X30

// I2C_bus 1 for PI
const int I2C_BUS = 1;

int i2c_handle;

// Buffers to read data and store vals
const int data_len = sizeof(float);
char read_buff[data_len];
float IMU_pitch;
float IMU_pitch_rate;
float mot_pos;
float mot_vel;
float mot_torque;

// I2C write cmd for mcu data
const uint8_t IMU_PITCH_CMD = 0x01;      // torso_pitch
const uint8_t IMU_PITCH_RATE_CMD = 0x02; // torso_pitch_rate
const uint8_t MOTOR_POS_CMD = 0x03;      // motor rotor position (not wheel)
const uint8_t MOTOR_VEL_CMD = 0x04;      // motor velocity position (not wheel)
const uint8_t MOTOR_TORQUE_CMD = 0X05;   // motor torque at motor shaft (not wheel)

// Heartbeat variables
const uint8_t HEARTBEAT_ID = 0xAA;
uint8_t heartbeat_ctr = 0;
uint8_t heartbeat_checksum = 0;

class MCUNode : public rclcpp::Node
{
public:
  MCUNode() : Node("mcu_node")
  {
    // call the reset mcu function to reset the mcu
    this->resetMCU();

    // Create publisher for "torso_angle" topic
    state_publisher_ = this->create_publisher<torsobot_interfaces::msg::TorsobotData>("torsobot_state", 10);

    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", I2C_BUS);

    // initialize i2c
    if ((i2c_handle = open(filename, O_RDWR)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error opening i2c, error: %d", i2c_handle);

      exitNode();
    }
    RCLCPP_INFO(this->get_logger(), "Opened i2c");

    // open I2C channel
    // Specify slave address
    if (ioctl(i2c_handle, I2C_SLAVE, PICO_SLAVE_ADDRESS) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device/slave!");
      close(i2c_handle);

      exitNode();
    }
    RCLCPP_INFO(this->get_logger(), "Successfully opened I2C device!");

    // heartbeat timer for 10ms (100Hz)
    heartbeat_timer_ = this->create_wall_timer(10ms, std::bind(&MCUNode::sendHearbeat, this)); // Use std::bind

    // mcu data timer for 10ms (100Hz)
    data_timer_ = this->create_wall_timer(10ms, std::bind(&MCUNode::dataCallback, this)); // Use std::bind
  }

private:
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr data_timer_;
  rclcpp::Publisher<torsobot_interfaces::msg::TorsobotData>::SharedPtr state_publisher_;
  gpiod::chip chip_;
  gpiod::line mcu_run_line_;
  int heartbeat_check;

  // timer callback for sending heartbeat on I2C
  // void heartbeatCallback(void)
  // {
  //   // Read / request sensor data from pico
  //   heartbeat_check = sendHearbeat();
  //   if (heartbeat_check == 0)
  //   {
  //     // Do nothing
  //   }
  //   else
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "Error sending heartbeat!");
  //     exitNode();
  //   }
  // };

  // timer callback for getting data on i2c
  void dataCallback(void)
  {
    // Read / request sensor data from pico
    int get_sensor_val_IMU_pitch = getSensorValue(IMU_PITCH_CMD, &IMU_pitch);
    int get_sensor_val_IMU_pitch_rate = getSensorValue(IMU_PITCH_RATE_CMD, &IMU_pitch_rate);
    int get_sensor_val_motor_pos = getSensorValue(MOTOR_POS_CMD, &mot_pos);
    int get_sensor_val_motor_vel = getSensorValue(MOTOR_VEL_CMD, &mot_vel);
    int get_sensor_val_motor_torque = getSensorValue(MOTOR_TORQUE_CMD, &mot_torque);

    // if sensor return values not 0
    bool sensor_val_okay = get_sensor_val_IMU_pitch || get_sensor_val_IMU_pitch_rate || get_sensor_val_motor_pos || get_sensor_val_motor_vel || get_sensor_val_motor_torque;
    sensor_val_okay = !sensor_val_okay;

    // if sensor_val_okay is 1
    if (sensor_val_okay)
    {
      auto message = torsobot_interfaces::msg::TorsobotData();
      message.torso_pitch = IMU_pitch;
      message.torso_pitch_rate = IMU_pitch_rate;
      message.motor_pos = mot_pos;
      message.motor_vel = mot_vel;
      message.motor_torque = mot_torque;

      RCLCPP_INFO(this->get_logger(), "IMU pitch: '%f'", message.torso_pitch);
      RCLCPP_INFO(this->get_logger(), "IMU pitch rate: '%f'", message.torso_pitch_rate);
      RCLCPP_INFO(this->get_logger(), "Motor position: '%f'", message.motor_pos);
      RCLCPP_INFO(this->get_logger(), "Motor velocity: '%f'", message.motor_vel);
      RCLCPP_INFO(this->get_logger(), "Motor torque: '%f'", message.motor_torque);

      this->state_publisher_->publish(message);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error reading sensor value");
    }
  };

  // Get sensor data over I2C
  int getSensorValue(uint8_t cmd, float *sensor_val_addr)
  {
    // Clear the read buffer
    memset(read_buff, 0, data_len);

    // Write command to slave
    if (write(i2c_handle, &cmd, sizeof(cmd)) != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C write failed!");
      return -1; // Indicate error
    }

    // Request sensor data from slave
    if (read(i2c_handle, read_buff, data_len) != data_len)
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C read failed!");
      return -2; // Indicate error
    }
    else
    {
      float temp_sensor_val; // temporary variable to check for nan
      memcpy(&temp_sensor_val, read_buff, data_len);
      if (!std::isnan(temp_sensor_val)) //if okay
      {
        memcpy(sensor_val_addr, read_buff, data_len);
      }
      else{
        RCLCPP_ERROR(this->get_logger(), "nan value received for %d!", cmd);
      }
    }

    return 0; // Success
  }

  //   Send heartbeat to pico
  void sendHearbeat(void)
  {
    uint8_t message[3];
    message[0] = HEARTBEAT_ID;                 // always constant
    message[1] = heartbeat_ctr;                // incremented by 1
    message[2] = heartbeat_ctr ^ HEARTBEAT_ID; // checksum to guard against data corrurption during i2c communication

    // write heatrbeat to i2c
    if (write(i2c_handle, &message, sizeof(message)) != sizeof(message))
    {
      RCLCPP_ERROR(this->get_logger(), "I2C write failed! ");
      exitNode();
      // return 1; // Indicate error
    }

    RCLCPP_INFO(this->get_logger(), "Sent hbeat: ID=0x%x, ctr=%d, csum=0x%x",
                (int)message[0], (int)message[1], (int)message[2]);

    heartbeat_ctr++;
    heartbeat_ctr = heartbeat_ctr % 256; // wrap around 255
    // return 0;
  }

  // Reset pico using the run pin
  void resetMCU(void)
  {
    chip_.open(GPIO_CHIP_NAME);
    mcu_run_line_ = chip_.get_line(RPI_GPIO_FOR_PICO_RUN);

    gpiod::line_request request_config;
    request_config.consumer = "ros2_mcu_run_toggle";                     // Set the consumer name
    request_config.request_type = gpiod::line_request::DIRECTION_OUTPUT; // Set the direction

    mcu_run_line_.request(request_config, true);
    mcu_run_line_.set_value(false); // set to low
    usleep(100 * 1000);             // 100 miliseconds
    mcu_run_line_.set_value(true);  // reset state
    usleep(3 * 1000 * 1000);        // wait for 3 seconds
  }

  void exitNode(void)
  {
    RCLCPP_FATAL(this->get_logger(), "Exiting node!");
    rclcpp::shutdown();
    exit(EXIT_FAILURE); // Terminate the program
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCUNode>());
  rclcpp::shutdown();
  return 0;
}
