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
#include <limits>    // for nan

#include "rclcpp/rclcpp.hpp"
#include "torsobot_interfaces/msg/torsobot_data.hpp"  //data from the torsobot including state variables
#include "torsobot_interfaces/msg/torsobot_state.hpp" // torsobor state variables only

// #include "torsobot_interfaces/srv/request_control_params.hpp" // service for requesting control parameters; not used

using namespace std::chrono_literals;

// Configuration for the GPIO pin
const std::string GPIO_CHIP_NAME = "gpiochip4"; // Verify with 'ls /dev/gpiochip*' on Pi5
const unsigned int RPI_GPIO_FOR_PICO_RUN = 23;  // GPIO23

// pico slave address
#define PICO_SLAVE_ADDRESS 0X30

// I2C_bus 1 for PI
const int I2C_BUS = 1;

int i2c_handle;

// Buffers to read data and store vals; only for 4 byte floats
const int float_len = sizeof(float);
char read_buff[float_len];
float torso_pitch, torso_pitch_rate;
float wheel_pos, wheel_vel, wheel_torque;
float mot_pos, mot_vel;
float wheel_cmd_torque;
int8_t mot_drv_mode;
int32_t signed_encoder_steps;

volatile float desired_torso_pitch;
volatile float wheel_max_torque, mot_max_torque, control_max_integral;
volatile float kp, ki, kd;

float torso_pitch_init = std::numeric_limits<float>::quiet_NaN(); //, mot_pos_init = std::numeric_limits<float>::quiet_NaN();
float wheel_rel_pos_init = std::numeric_limits<float>::quiet_NaN();

// I2C write cmd for mcu data
const uint8_t DESIRED_TORSO_PITCH_CMD = 0x00; // desired torso_pitch
const uint8_t TORSO_PITCH_CMD = 0x01;         // torso_pitch
const uint8_t TORSO_PITCH_RATE_CMD = 0x02;    // torso_pitch_rate
const uint8_t WHEEL_POS_CMD = 0x03;           // motor rotor position (not wheel)
const uint8_t WHEEL_VEL_CMD = 0x04;           // motor velocity position (not wheel)
const uint8_t WHEEL_TORQUE_CMD = 0X05;        // motor torque at motor shaft (not wheel)
const uint8_t MOTOR_DRV_MODE_CMD = 0X06;      // motor driver mode

const uint8_t WHEEL_MAX_TORQUE_CMD = 0X07;  // motor max torque value
const uint8_t KP_CMD = 0X08;                // Kp value
const uint8_t KI_CMD = 0X09;                // Ki value
const uint8_t KD_CMD = 0X0A;                // Kd value
const uint8_t CTRL_MAX_INTEGRAL_CMD = 0X0B; // max integral term
const uint8_t WHEEL_CMD_TORQUE_CMD = 0X0C;  // commanded motor torque

const uint8_t MOT_POS_CMD = 0X0D; // mot_pos
const uint8_t MOT_VEL_CMD = 0X0E; // mot_vel

// for the initial values
const uint8_t WHEEL_REL_POS_INIT_CMD = 0X0F;
const uint8_t TORSO_PITCH_INIT_CMD = 0X10;

// controller type
const uint8_t CONTROLLER_CMD = 0X11;
volatile int controller = 1; // default is 1, but this is overwritten from the param file

// encoder steps cmd
const uint8_t ENCODER_STEPS_CMD = 0x12; // signed encoder steps from the mcu

// Heartbeat variables
const uint8_t HEARTBEAT_ID = 0xAA;
uint8_t heartbeat_ctr = 0;
uint8_t heartbeat_checksum = 0;

int bytes_written;

// counter for the nan values encountered udring I2C receive
uint64_t nan_ctr = 0;

// counter for the callback function
uint64_t data_callback_ctr = 0;

// int check received from the pico if the robot has went into exit state
volatile uint8_t has_robot_stopped = 0x00;

class MCUNode : public rclcpp::Node
{
public:
  MCUNode() : Node("mcu_node")
  {
    // call the reset mcu function to reset the mcu
    RCLCPP_INFO(this->get_logger(), "Resetting the MCU...");
    this->resetMCU();
    RCLCPP_INFO(this->get_logger(), "Reset the MCU!");

    // Create publisher for "torsobot_state" topic
    state_publisher_ = this->create_publisher<torsobot_interfaces::msg::TorsobotState>("torsobot_state", 10);

    // Create publisher for "torsobot_state" topic
    data_publisher_ = this->create_publisher<torsobot_interfaces::msg::TorsobotData>("torsobot_data", 10);

    // Create service server for one-time values
    // control_params_server_ = this->create_service<torsobot_interfaces::srv::RequestControlParams>("control_params", std::bind(&MCUNode::handleServiceRequest, this, std::placeholders::_1, std::placeholders::_2));

    // Create ROS2 parameters
    this->declare_parameter("desired_torso_pitch", 180.0); // default value of 180 in degrees
    this->declare_parameter("kp", 0.0);                    // default value of 0
    this->declare_parameter("ki", 0.0);                    // default value of 0
    this->declare_parameter("kd", 0.0);                    // default value of 0
    this->declare_parameter("wheel_max_torque", 0.0);      // default value of 0
    this->declare_parameter("control_max_integral", 0.0);  // default value of 0
    this->declare_parameter("controller", 1);              // default value of 1 (GCPD controller)

    // get ROS2 parameters from the param file (listed in launch file) and assign to these variables (2nd argument)
    this->get_parameter("desired_torso_pitch", desired_torso_pitch);
    this->get_parameter("kp", kp);
    this->get_parameter("ki", ki);
    this->get_parameter("kd", kd);
    this->get_parameter("wheel_max_torque", wheel_max_torque);
    this->get_parameter("control_max_integral", control_max_integral);
    this->get_parameter("controller", controller);

    RCLCPP_INFO(this->get_logger(), "Desired torso pitch is %f: ", desired_torso_pitch);
    RCLCPP_INFO(this->get_logger(), "kp %f: ", kp);
    RCLCPP_INFO(this->get_logger(), "ki %f: ", ki);
    RCLCPP_INFO(this->get_logger(), "kd %f: ", kd);
    RCLCPP_INFO(this->get_logger(), "wheel_max_torque %f: ", wheel_max_torque);
    RCLCPP_INFO(this->get_logger(), "control_max_integral %f: ", control_max_integral);
    RCLCPP_INFO(this->get_logger(), "controller %d: ", controller);

    // for the i2c bus
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

    // write the desired values to pico
    sendDataToMCU(DESIRED_TORSO_PITCH_CMD, &desired_torso_pitch);
    sendDataToMCU(CONTROLLER_CMD, &controller);
    sendDataToMCU(KP_CMD, &kp);
    sendDataToMCU(KI_CMD, &ki);
    sendDataToMCU(KD_CMD, &kd);
    sendDataToMCU(WHEEL_MAX_TORQUE_CMD, &wheel_max_torque);
    sendDataToMCU(CTRL_MAX_INTEGRAL_CMD, &control_max_integral);

    // heartbeat timer for 10ms (100Hz)
    heartbeat_timer_ = this->create_wall_timer(50ms, std::bind(&MCUNode::sendHeartbeat, this)); // Use std::bind

    // mcu data timer for 10ms (100Hz)
    data_timer_ = this->create_wall_timer(10ms, std::bind(&MCUNode::i2cDataCallback, this)); // Use std::bind

    RCLCPP_INFO(this->get_logger(), "Starting node!");
  }

private:
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr data_timer_;
  rclcpp::Publisher<torsobot_interfaces::msg::TorsobotState>::SharedPtr state_publisher_;
  rclcpp::Publisher<torsobot_interfaces::msg::TorsobotData>::SharedPtr data_publisher_;
  // rclcpp::Service<torsobot_interfaces::srv::RequestControlParams>::SharedPtr control_params_server_;

  bool read_control_params_ = false;

  gpiod::chip chip_;
  gpiod::line mcu_run_line_;

  // send desired values to the pico
  void sendDataToMCU(uint8_t cmd, volatile float *sensor_val_addr)
  {
    char data_write_buff[sizeof(_Float32) + 1]; // 4 bytes for the data and 1 for the cmd
    data_write_buff[0] = cmd;
    memcpy(&data_write_buff[1], const_cast<float *>(sensor_val_addr), sizeof(_Float32));

    int write_result = write(i2c_handle, &data_write_buff, sizeof(data_write_buff));

    if (write_result != sizeof(data_write_buff))
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending data to MCU for %d", cmd);
      exitNode();
    }
  }

  // send desired values to the pico (int version)
  void sendDataToMCU(uint8_t cmd, volatile int *sensor_val_addr)
  {
    char data_write_buff[sizeof(int) + 1]; // 4 bytes for the data and 1 for the cmd
    data_write_buff[0] = cmd;
    memcpy(&data_write_buff[1], const_cast<int *>(sensor_val_addr), sizeof(int));

    int write_result = write(i2c_handle, &data_write_buff, sizeof(data_write_buff));

    if (write_result != sizeof(data_write_buff))
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending data to MCU for %d", cmd);
      exitNode();
    }
  }

  // timer callback for getting data on i2c
  void i2cDataCallback(void)
  {

    // print a warning for nan values
    if (nan_ctr >= 100)
    {
      // RCLCPP_WARN(this->get_logger(), "100 nan values found!");
      // exitNode();
    }

    // Read / request sensor data from pico
    int get_sensor_val_torso_pitch = getSensorValue(TORSO_PITCH_CMD, &torso_pitch);
    int get_sensor_val_torso_pitch_rate = getSensorValue(TORSO_PITCH_RATE_CMD, &torso_pitch_rate);
    int get_sensor_val_wheel_pos = getSensorValue(WHEEL_POS_CMD, &wheel_pos);
    int get_sensor_val_wheel_vel = getSensorValue(WHEEL_VEL_CMD, &wheel_vel);
    int get_sensor_val_wheel_torque = getSensorValue(WHEEL_TORQUE_CMD, &wheel_torque);
    int get_sensor_val_motor_drv_mode = getSensorValue(MOTOR_DRV_MODE_CMD, &mot_drv_mode);
    int get_sensor_val_wheel_cmd_torque = getSensorValue(WHEEL_CMD_TORQUE_CMD, &wheel_cmd_torque);
    int get_encoder_steps = getSensorValue(ENCODER_STEPS_CMD, &signed_encoder_steps);

    // (void)getSensorValue(MOT_POS_CMD, &mot_pos);
    // (void)getSensorValue(MOT_VEL_CMD, &mot_vel);

    // after about 2000ms
    if (data_callback_ctr == 350)
    {
      // get this value only once
      (void)getSensorValue(TORSO_PITCH_INIT_CMD, &torso_pitch_init);
      (void)getSensorValue(WHEEL_REL_POS_INIT_CMD, &wheel_rel_pos_init);
      // (void)getSensorValue(MOT_POS_INIT_CMD, &mot_pos_init);
    }

    // check for position mode
    if (mot_drv_mode != 10)
    {
      RCLCPP_ERROR(this->get_logger(), "Driver mode is not position mode!");
      // exitNode();
    }

    // if sensor return values not 0
    bool sensor_val_okay = get_sensor_val_torso_pitch || get_sensor_val_torso_pitch_rate || get_sensor_val_wheel_pos || get_sensor_val_wheel_vel || get_sensor_val_wheel_torque || get_sensor_val_motor_drv_mode || get_sensor_val_wheel_cmd_torque || get_encoder_steps;
    sensor_val_okay = !sensor_val_okay; // invert logic

    // if sensor_val_okay is 1
    if (sensor_val_okay)
    {
      auto state_message = torsobot_interfaces::msg::TorsobotState();
      state_message.torso_pitch = torso_pitch;
      state_message.torso_pitch_rate = torso_pitch_rate;
      state_message.wheel_pos = wheel_pos;
      state_message.wheel_vel = wheel_vel;
      // state_message.wheel_torque = wheel_torque;
      // state_message.motor_drv_mode = mot_drv_mode;

      this->state_publisher_->publish(state_message);

      auto data_message = torsobot_interfaces::msg::TorsobotData();
      data_message.torsobot_state = state_message;
      // data_message.torsobot_state.torso_pitch = torso_pitch;
      // data_message.torsobot_state.torso_pitch_rate = torso_pitch_rate;
      // data_message.torsobot_state.wheel_pos = wheel_pos;
      // data_message.torsobot_state.wheel_vel = wheel_vel;
      data_message.wheel_torque = wheel_torque;
      data_message.mot_drv_mode = mot_drv_mode;
      data_message.wheel_cmd_torque = wheel_cmd_torque;
      // data_message.mot_pos = mot_pos;
      // data_message.mot_vel = mot_vel;
      data_message.torso_pitch_init = torso_pitch_init;
      data_message.wheel_rel_pos_init = wheel_rel_pos_init;
      data_message.encoder_steps = signed_encoder_steps;
      // data_message.mot_pos_init = mot_pos_init;

      this->data_publisher_->publish(data_message);

      RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f, %f, %f, %d, %d", torso_pitch, torso_pitch_rate, wheel_pos, wheel_vel, wheel_cmd_torque, wheel_torque, mot_drv_mode, signed_encoder_steps);

      // RCLCPP_INFO(this->get_logger(), "IMU pitch: '%f'", torso_pitch);
      // RCLCPP_INFO(this->get_logger(), "IMU pitch rate: '%f'", torso_pitch_rate);
      // RCLCPP_INFO(this->get_logger(), "Wheel position: '%f'", wheel_pos);
      // RCLCPP_INFO(this->get_logger(), "Wheel velocity: '%f'", wheel_vel);
      // RCLCPP_INFO(this->get_logger(), "Wheel torque: '%f'", wheel_torque);
      // RCLCPP_INFO(this->get_logger(), "Wheel cmd torque: '%f'", wheel_cmd_torque);
      // RCLCPP_INFO(this->get_logger(), "Motor driver mode: '%d'", mot_drv_mode);
      // RCLCPP_INFO(this->get_logger(), "Torso init pitch: '%f'", torso_pitch_init);
      // RCLCPP_INFO(this->get_logger(), "Motor init pos: '%f'", mot_pos_init);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error reading sensor value");
    }

    // increment data_callback_ctr
    data_callback_ctr++;
  }

  // Get sensor data over I2C
  int getSensorValue(uint8_t cmd, float *sensor_val_addr)
  {
    // Clear the read buffer
    memset(read_buff, 0, float_len);

    // Write command to slave
    if (write(i2c_handle, &cmd, sizeof(cmd)) != sizeof(cmd))
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C write failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -1; // Indicate error
    }

    // Request sensor data from slave
    if (read(i2c_handle, read_buff, float_len) != float_len)
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C read failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -2; // Indicate error
    }
    else
    {
      // float temp_sensor_val; // temporary variable to check for nan
      // memcpy(&temp_sensor_val, read_buff, float_len);
      memcpy(sensor_val_addr, read_buff, float_len);
      if (std::isnan(*sensor_val_addr)) // if okay
      {

        RCLCPP_ERROR(this->get_logger(), "nan value received for %d!", cmd);
        nan_ctr++;
      }
    }

    return 0; // Success
  }

  // Get sensor data over I2C for int data types
  int getSensorValue(uint8_t cmd, int8_t *sensor_val_addr_int)
  {
    // Clear the read buffer
    memset(read_buff, 0, float_len);

    // Write command to slave
    if (write(i2c_handle, &cmd, sizeof(cmd)) != sizeof(cmd))
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C write failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -1; // Indicate error
    }

    // Request sensor data from slave
    if (read(i2c_handle, read_buff, float_len) != float_len)
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

  // Get sensor data over I2C for int data types
  int getSensorValue(uint8_t cmd, int32_t *sensor_val_addr_int)
  {
    // Clear the read buffer
    memset(read_buff, 0, float_len);

    // Write command to slave
    if (write(i2c_handle, &cmd, sizeof(cmd)) != sizeof(cmd))
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C write failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -1; // Indicate error
    }

    // Request sensor data from slave
    if (read(i2c_handle, read_buff, float_len) != float_len)
    {
      RCLCPP_ERROR(this->get_logger(), "sensor value I2C read failed for %d! %s (%d)", cmd, strerror(errno), errno);
      return -2; // Indicate error
    }
    else
    {
      memcpy(sensor_val_addr_int, read_buff, sizeof(int32_t));
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

    // Request sensor data from slave
    if (read(i2c_handle, (void *)&has_robot_stopped, sizeof(uint8_t)) == sizeof(uint8_t))
    {
      RCLCPP_INFO(this->get_logger(), "Robot running code: 0x%02X", has_robot_stopped);

      if (has_robot_stopped == 0xFF || has_robot_stopped == 0xAA || has_robot_stopped == 0xBB)
      {
        // robot has gone into while loop
        RCLCPP_ERROR(this->get_logger(), "Robot has entered the exit while loop!");
        exitNode();
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "heartbeat I2C read failed! %s (%d)", strerror(errno), errno);
      // exitNode();
      // return -2; // Indicate error
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
    usleep(2 * 1000 * 1000);        // wait for 2 seconds for arduino loop to start before I2C
  }

  void exitNode(void)
  {
    RCLCPP_FATAL(this->get_logger(), "Exiting node!");
    rclcpp::shutdown();
    exit(EXIT_FAILURE); // Terminate the program
  }
};

// Main
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCUNode>());
  rclcpp::shutdown();
  return 0;
}