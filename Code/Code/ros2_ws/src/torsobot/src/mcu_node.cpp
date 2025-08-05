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
#include "torsobot_interfaces/msg/torsobot_state.hpp"

#include "torsobot_interfaces/srv/request_control_params.hpp"

using namespace std::chrono_literals;

// Configuration for the GPIO pin
const std::string GPIO_CHIP_NAME = "gpiochip0"; // Verify with 'ls /dev/gpiochip*' on your Pi5
const unsigned int RPI_GPIO_FOR_PICO_RUN = 14;  // GPIO14

// pico slave address
#define PICO_SLAVE_ADDRESS 0X30

// I2C_bus 1 for PI
const int I2C_BUS = 1;

int i2c_handle;

// Buffers to read data and store vals; only for 4 byte floats
const int data_len = sizeof(float);
char read_buff[data_len];
float IMU_pitch;
float IMU_pitch_rate;
float mot_pos;
float mot_vel;
float mot_torque;
int8_t mot_drv_mode;
float desired_torso_pitch;
float mot_max_torque, mot_cmd_torque;
float kp, ki, kd;

// I2C write cmd for mcu data
const uint8_t DESIRED_TORSO_PITCH_CMD = 0x00; // desired torso_pitch
const uint8_t IMU_PITCH_CMD = 0x01;           // torso_pitch
const uint8_t IMU_PITCH_RATE_CMD = 0x02;      // torso_pitch_rate
const uint8_t MOTOR_POS_CMD = 0x03;           // motor rotor position (not wheel)
const uint8_t MOTOR_VEL_CMD = 0x04;           // motor velocity position (not wheel)
const uint8_t MOTOR_TORQUE_CMD = 0X05;        // motor torque at motor shaft (not wheel)
const uint8_t MOTOR_DRV_MODE_CMD = 0X06;      // motor driver mode

const uint8_t MOTOR_MAX_TORQUE_CMD = 0X07; // motor max torque value
const uint8_t KP_CMD = 0X08;               // Kp value
const uint8_t KI_CMD = 0X09;               // Ki value
const uint8_t KD_CMD = 0X0A;               // Kd value
const uint8_t MOTOR_CMD_TORQUE_CMD = 0X0B; // commanded motor torque

// Heartbeat variables
const uint8_t HEARTBEAT_ID = 0xAA;
uint8_t heartbeat_ctr = 0;
uint8_t heartbeat_checksum = 0;

int bytes_written;

class MCUNode : public rclcpp::Node
{
public:
  MCUNode() : Node("mcu_node")
  {
    // call the reset mcu function to reset the mcu
    this->resetMCU();

    // Create publisher for "torsobot_state" topic
    state_publisher_ = this->create_publisher<torsobot_interfaces::msg::TorsobotState>("torsobot_state", 10);

    // Create publisher for "torsobot_state" topic
    data_publisher_ = this->create_publisher<torsobot_interfaces::msg::TorsobotData>("torsobot_data", 10);

    // Create service server for one-time values
    control_params_server_ = this->create_service<torsobot_interfaces::srv::RequestControlParams>("control_params", std::bind(&MCUNode::handleServiceRequest, this, std::placeholders::_1, std::placeholders::_2));

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
    heartbeat_timer_ = this->create_wall_timer(25ms, std::bind(&MCUNode::sendHeartbeat, this)); // Use std::bind

    // mcu data timer for 10ms (100Hz)
    data_timer_ = this->create_wall_timer(25ms, std::bind(&MCUNode::i2cDataCallback, this)); // Use std::bind

    RCLCPP_INFO(this->get_logger(), "Starting node!");
  }

private:
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr data_timer_;
  rclcpp::Publisher<torsobot_interfaces::msg::TorsobotState>::SharedPtr state_publisher_;
  rclcpp::Publisher<torsobot_interfaces::msg::TorsobotData>::SharedPtr data_publisher_;
  rclcpp::Service<torsobot_interfaces::srv::RequestControlParams>::SharedPtr control_params_server_;

  bool read_control_params_ = false;

  gpiod::chip chip_;
  gpiod::line mcu_run_line_;
  // int heartbeat_check;

  // timer callback for sending heartbeat on I2C
  // void heartbeatCallback(void)
  // {
  //   // Read / request sensor data from pico
  //   heartbeat_check = sendHeartbeat();
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

  // Service callback for the control parameters
  void handleServiceRequest(
      const std::shared_ptr<torsobot_interfaces::srv::RequestControlParams::Request> request,
      std::shared_ptr<torsobot_interfaces::srv::RequestControlParams::Response> response)
  {
    (void)request; // Suppress unused parameter warning
    response->desired_torso_pitch = desired_torso_pitch;
    response->mot_max_torque = mot_max_torque;
    response->kp = kp;
    response->ki = ki;
    response->kd = kd;
  }

  // timer callback for getting data on i2c
  void i2cDataCallback(void)
  {
    if (!read_control_params_)
    {
      // request one-time data from the pico
      // desired imu_angle
      (void)getSensorValue(DESIRED_TORSO_PITCH_CMD, &desired_torso_pitch);

      // max_torque value
      (void)getSensorValue(MOTOR_MAX_TORQUE_CMD, &mot_max_torque);

      // PID control gain values
      (void)getSensorValue(KP_CMD, &kp);
      (void)getSensorValue(KI_CMD, &ki);
      (void)getSensorValue(KD_CMD, &kd);

      RCLCPP_INFO(this->get_logger(), "desired_torso_pitch: %f", desired_torso_pitch);
      RCLCPP_INFO(this->get_logger(), "motor_max_torque: %f", mot_max_torque);
      RCLCPP_INFO(this->get_logger(), "kp: %f, ki: %f, kd: %f", kp, ki, kd);

      // if all three values are zero, then values weren't received properly
      if (kp || ki || kd)
      {
        read_control_params_ = true;
      }
    }

    // Read / request sensor data from pico
    int get_sensor_val_IMU_pitch = getSensorValue(IMU_PITCH_CMD, &IMU_pitch);
    int get_sensor_val_IMU_pitch_rate = getSensorValue(IMU_PITCH_RATE_CMD, &IMU_pitch_rate);
    int get_sensor_val_motor_pos = getSensorValue(MOTOR_POS_CMD, &mot_pos);
    int get_sensor_val_motor_vel = getSensorValue(MOTOR_VEL_CMD, &mot_vel);
    int get_sensor_val_motor_torque = getSensorValue(MOTOR_TORQUE_CMD, &mot_torque);
    int get_sensor_val_motor_drv_mode = getSensorValue(MOTOR_DRV_MODE_CMD, &mot_drv_mode);
    int get_sensor_val_motor_cmd_torque = getSensorValue(MOTOR_CMD_TORQUE_CMD, &mot_cmd_torque);

    if (mot_drv_mode == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Driver mode is 0");
      // exitNode();
    }

    // if sensor return values not 0
    bool sensor_val_okay = get_sensor_val_IMU_pitch || get_sensor_val_IMU_pitch_rate || get_sensor_val_motor_pos || get_sensor_val_motor_vel || get_sensor_val_motor_torque || get_sensor_val_motor_drv_mode || get_sensor_val_motor_cmd_torque;
    sensor_val_okay = !sensor_val_okay; // invert logic

    // if sensor_val_okay is 1
    if (sensor_val_okay)
    {
      auto state_message = torsobot_interfaces::msg::TorsobotState();
      state_message.torso_pitch = IMU_pitch;
      state_message.torso_pitch_rate = IMU_pitch_rate;
      state_message.motor_pos = mot_pos;
      state_message.motor_vel = mot_vel;
      // state_message.motor_torque = mot_torque;
      // state_message.motor_drv_mode = mot_drv_mode;

      this->state_publisher_->publish(state_message);

      auto data_message = torsobot_interfaces::msg::TorsobotData();
      data_message.torso_pitch = IMU_pitch;
      data_message.torso_pitch_rate = IMU_pitch_rate;
      data_message.motor_pos = mot_pos;
      data_message.motor_vel = mot_vel;
      data_message.motor_torque = mot_torque;
      data_message.motor_drv_mode = mot_drv_mode;
      data_message.motor_cmd_torque = mot_cmd_torque;

      this->data_publisher_->publish(data_message);

      RCLCPP_INFO(this->get_logger(), "IMU pitch: '%f'", IMU_pitch);
      RCLCPP_INFO(this->get_logger(), "IMU pitch rate: '%f'", IMU_pitch_rate);
      RCLCPP_INFO(this->get_logger(), "Motor position: '%f'", mot_pos);
      RCLCPP_INFO(this->get_logger(), "Motor velocity: '%f'", mot_vel);
      RCLCPP_INFO(this->get_logger(), "Motor torque: '%f'", mot_torque);
      RCLCPP_INFO(this->get_logger(), "Motor cmd torque: '%f'", mot_cmd_torque);
      RCLCPP_INFO(this->get_logger(), "Motor driver mode: '%d'", mot_drv_mode);
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
    if (write(i2c_handle, &cmd, sizeof(cmd)) != sizeof(cmd))
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C write failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -1; // Indicate error
    }

    // Request sensor data from slave
    if (read(i2c_handle, read_buff, data_len) != data_len)
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C read failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -2; // Indicate error
    }
    else
    {
      float temp_sensor_val; // temporary variable to check for nan
      memcpy(&temp_sensor_val, read_buff, data_len);
      if (!std::isnan(temp_sensor_val)) // if okay
      {
        memcpy(sensor_val_addr, read_buff, data_len);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "nan value received for %d!", cmd);
      }
    }

    return 0; // Success
  }

  // Get sensor data over I2C for int data types
  int getSensorValue(uint8_t cmd, int8_t *sensor_val_addr_int)
  {
    // Clear the read buffer
    memset(read_buff, 0, data_len);

    // Write command to slave
    if (write(i2c_handle, &cmd, sizeof(cmd)) != sizeof(cmd))
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C write failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -1; // Indicate error
    }

    // Request sensor data from slave
    if (read(i2c_handle, read_buff, data_len) != data_len)
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C read failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -2; // Indicate error
    }
    else
    {
      memcpy(sensor_val_addr_int, read_buff, sizeof(uint8_t));
    }

    return 0; // Success
  }

  //   Send heartbeat to pico
  void sendHeartbeat(void)
  {
    uint8_t hbt_message[3];
    hbt_message[0] = HEARTBEAT_ID;                 // always constant
    hbt_message[1] = heartbeat_ctr;                // incremented by 1
    hbt_message[2] = heartbeat_ctr ^ HEARTBEAT_ID; // checksum to guard against data corrurption during i2c communication

    // write heatrbeat to i2c
    if (write(i2c_handle, &hbt_message, sizeof(hbt_message)) != sizeof(hbt_message))
    {
      RCLCPP_ERROR(this->get_logger(), "I2C heartbeat write failed! ");
      exitNode();
      // return 1; // Indicate error
    }

    RCLCPP_INFO(this->get_logger(), "Sent hbeat: ID=0x%x, ctr=%d, csum=0x%x",
                (int)hbt_message[0], (int)hbt_message[1], (int)hbt_message[2]);

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
    usleep(1 * 1000 * 1000);        // wait for 1 second for arduino loop to start before I2C
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
