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

    // Create ROS2 parameters
    this->declare_parameter("desired_torso_pitch_", 180.0); // default value of 180 in degrees
    this->declare_parameter("kp_", 0.0);                    // default value of 0
    this->declare_parameter("ki_", 0.0);                    // default value of 0
    this->declare_parameter("kd_", 0.0);                    // default value of 0
    this->declare_parameter("wheel_max_torque_", 0.0);      // default value of 0
    this->declare_parameter("control_max_integral_", 0.0);  // default value of 0
    this->declare_parameter("controller", 1);               // default value of 1 (GCPD controller)

    // get ROS2 parameters from the param file (listed in launch file) and assign to these variables (2nd argument)
    // Create temporary variables for parameter retrieval
    float temp_pitch, temp_kp, temp_ki, temp_kd, temp_wheel, temp_max_integral;
    int temp_controller; // ROS2 parameter integers usually bind to int, not uint8_t directly

    // Get parameters into the safe temporary variables
    this->get_parameter("desired_torso_pitch_", temp_pitch);
    this->get_parameter("kp_", temp_kp);
    this->get_parameter("ki_", temp_ki);
    this->get_parameter("kd_", temp_kd);
    this->get_parameter("wheel_max_torque_", temp_wheel);
    this->get_parameter("control_max_integral_", temp_max_integral);
    this->get_parameter("controller", temp_controller);

    // Copy the values into the packed struct
    config_data_.desired_torso_pitch = temp_pitch;
    config_data_.kp = temp_kp;
    config_data_.ki = temp_ki;
    config_data_.kd = temp_kd;
    config_data_.wheel_max_torque = temp_wheel;
    config_data_.control_max_integral = temp_max_integral;
    config_data_.controller = static_cast<uint8_t>(temp_controller); // Safe cast to 8-bit

    RCLCPP_INFO(this->get_logger(), "Desired torso pitch is %f: ", config_data_.desired_torso_pitch);
    RCLCPP_INFO(this->get_logger(), "kp_ %f: ", config_data_.kp);
    RCLCPP_INFO(this->get_logger(), "ki_ %f: ", config_data_.ki);
    RCLCPP_INFO(this->get_logger(), "kd_ %f: ", config_data_.kd);
    RCLCPP_INFO(this->get_logger(), "wheel_max_torque_ %f: ", config_data_.wheel_max_torque);
    RCLCPP_INFO(this->get_logger(), "control_max_integral_ %f: ", config_data_.control_max_integral);
    RCLCPP_INFO(this->get_logger(), "controller %d: ", config_data_.controller);

    // for the i2c bus
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", I2C_BUS);

    // initialize i2c
    if ((i2c_handle_ = open(filename, O_RDWR)) < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Error opening i2c, error: %d", i2c_handle_);
      exit_node();
    }
    RCLCPP_INFO(this->get_logger(), "Opened I2C handle");

    // open I2C channel
    // Specify slave address
    if (ioctl(i2c_handle_, I2C_SLAVE, PICO_SLAVE_ADDRESS) < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to open I2C device/slave!");
      close(i2c_handle_);

      exit_node();
    }
    RCLCPP_INFO(this->get_logger(), "Successfully opened I2C device!");

    // write the desired values to pico
    int result = i2c_transaction<mcuConfigData, uint8_t>(config_data_, &config_ack_);
    if (result == 1)
    {
      RCLCPP_FATAL(this->get_logger(), "Could not write config data to mcu!");
      exit_node();
    }

    if (config_ack_ == 0)
    {
      RCLCPP_FATAL(this->get_logger(), "mcu did not acknowledge config write!");
      exit_node();
    }

    // heartbeat timer for 10ms (100Hz)
    heartbeat_timer_ = this->create_wall_timer(50ms, std::bind(&MCUNode::send_heartbeat, this)); // Use std::bind

    // mcu data timer for 10ms (100Hz)
    data_timer_ = this->create_wall_timer(10ms, std::bind(&MCUNode::i2c_data_callback, this)); // Use std::bind

    RCLCPP_INFO(this->get_logger(), "Starting node!");
  }

private:
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr data_timer_;
  rclcpp::Publisher<torsobot_interfaces::msg::TorsobotState>::SharedPtr state_publisher_;
  rclcpp::Publisher<torsobot_interfaces::msg::TorsobotData>::SharedPtr data_publisher_;

  gpiod::chip chip_;
  gpiod::line mcu_run_line_;

  // --- Variables --- //
  // Configuration for the GPIO pin
  const std::string GPIO_CHIP_NAME = "gpiochip4"; // Verify with 'ls /dev/gpiochip*' on Pi5
  const unsigned int RPI_GPIO_FOR_PICO_RUN = 22;  // GPIO22; changed from 23 because i did not want to change the connections manually

  // I2C variables
  static constexpr int PICO_SLAVE_ADDRESS = 0X30;
  static constexpr int I2C_BUS = 1;
  int i2c_handle_;

  // i2c command variables
  static constexpr uint8_t HEARTBEAT_ID_ = 0XAA;
  static constexpr uint8_t MCU_CONTROLLER_CONFIG_ID_ = 0XBB;
  static constexpr uint8_t INIT_DATA_ID_ = 0XCC;
  static constexpr uint8_t STATE_DATA_ID = 0XDD;

  // i2c structs
  // hearbeat
  struct __attribute__((packed)) Heartbeat
  {
    const uint8_t id = HEARTBEAT_ID_;
    uint8_t counter;
    uint8_t checksum;
  };

  // config data
  struct __attribute__((packed)) mcuConfigData
  {
    const uint8_t id = MCU_CONTROLLER_CONFIG_ID_;
    float desired_torso_pitch;
    float kp;
    float ki;
    float kd;
    float wheel_max_torque;
    float control_max_integral;
    uint8_t controller;
    uint8_t checksum;
  } config_data_;

  // init data
  struct __attribute__((packed)) InitData
  {
    float torso_pitch_init = std::numeric_limits<float>::quiet_NaN();
    float wheel_rel_pos_init = std::numeric_limits<float>::quiet_NaN();
    uint8_t checksum;
  } init_data_;

  // robot data
  struct __attribute__((packed)) StateData
  {
    float torso_pitch;
    float torso_pitch_rate;
    float wheel_pos;
    float wheel_vel;
    int32_t encoder_steps;
    float encoder_ang;
    float encoder_speed;
    uint8_t motor_drv_mode;
    float wheel_cmd_torque;
    float wheel_actual_torque;
    uint8_t checksum;
  };

  // config data okay
  uint8_t config_ack_ = 0;

  // robot status
  uint8_t robot_status_;

  // counter for the nan values encountered udring I2C receive
  uint64_t nan_ctr = 0;

  // counter for the callback function
  uint64_t data_callback_ctr = 0;

  // counter for heartbeat
  uint64_t heartbeat_ctr_ = 0;

  // timer callback for getting data on i2c
  void i2c_data_callback(void)
  {
    StateData state_data_;
    // print a warning for nan values
    if (nan_ctr >= 100)
    {
      // RCLCPP_WARN(this->get_logger(), "100 nan values found!");
      // exit_node();
    }

    // Read / request sensor data from pico
    int result = i2c_transaction<uint8_t, StateData>(STATE_DATA_ID, &state_data_);

    if (result == 1)
    {
      RCLCPP_FATAL(this->get_logger(), "Could not write data cmd to mcu!");
      exit_node();
    }
    else if (result == 2)
    {
      RCLCPP_FATAL(this->get_logger(), "Could not read state data from mcu!");
      exit_node();
    }

    // after about 2000ms
    if (data_callback_ctr++ == 350)
    {
      // read init data
      int result = i2c_transaction<uint8_t, InitData>(INIT_DATA_ID_, &init_data_);

      if (result == 1)
      {
        RCLCPP_FATAL(this->get_logger(), "Could not write data cmd to mcu!");
        exit_node();
      }
      else if (result == 2)
      {
        RCLCPP_FATAL(this->get_logger(), "Could not read state data from mcu!");
        exit_node();
      }

      uint8_t init_data_calculated_checksum = calculate_checksum<InitData>(init_data_);

      bool init_data_checksum_okay = init_data_calculated_checksum == init_data_.checksum;

      // if received value checksum is okay
      if (!init_data_checksum_okay)
      {
        RCLCPP_FATAL(this->get_logger(), "Checksum invalid for init data from mcu");
        exit_node();
      }
    }

    // check for position mode
    if (state_data_.motor_drv_mode != 10)
    {
      RCLCPP_ERROR(this->get_logger(), "Driver mode is not position mode!");
      // exit_node();
    }

    //  check checksum
    uint8_t calculated_checksum = calculate_checksum<StateData>(state_data_);

    bool state_checksum_okay = calculated_checksum == state_data_.checksum;
    // if sensor_val_okay is 1
    if (state_checksum_okay)
    {
      auto state_message = torsobot_interfaces::msg::TorsobotState();
      state_message.torso_pitch = state_data_.torso_pitch;
      state_message.torso_pitch_rate = state_data_.torso_pitch_rate;
      state_message.wheel_pos = state_data_.wheel_pos;
      state_message.wheel_vel = state_data_.wheel_vel;

      this->state_publisher_->publish(state_message);

      auto data_message = torsobot_interfaces::msg::TorsobotData();
      data_message.torsobot_state = state_message;

      data_message.wheel_torque = state_data_.wheel_actual_torque;
      data_message.mot_drv_mode = state_data_.motor_drv_mode;
      data_message.wheel_cmd_torque = state_data_.wheel_cmd_torque;
      data_message.torso_pitch_init = init_data_.torso_pitch_init;
      data_message.wheel_rel_pos_init = init_data_.wheel_rel_pos_init;
      data_message.encoder_steps = state_data_.encoder_steps;
      // data_message.mot_pos_init = mot_pos_init;

      this->data_publisher_->publish(data_message);

      RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f, %f, %f, %d, %d", state_data_.torso_pitch, state_data_.torso_pitch_rate, state_data_.wheel_pos, state_data_.wheel_vel, state_data_.wheel_cmd_torque, state_data_.wheel_actual_torque, state_data_.motor_drv_mode, state_data_.encoder_steps);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Checksum invalid for state data from mcu");
    }
  }

  //   Send heartbeat to pico
  void send_heartbeat(void)
  {
    Heartbeat heartbeat_;
    // no need to write heartbeat id since it is always constant
    heartbeat_.counter = heartbeat_ctr_;                  // incremented by 1
    heartbeat_.checksum = heartbeat_ctr_ ^ HEARTBEAT_ID_; // checksum to guard against data corrurption during i2c communication

    // write heatrbeat to i2c
    int result = i2c_transaction<Heartbeat, uint8_t>(heartbeat_, &robot_status_);

    if (result == 1)
    {
      RCLCPP_FATAL(this->get_logger(), "Could not write heartbeat to mcu!");
      exit_node();
    }
    else if (result == 2)
    {
      RCLCPP_FATAL(this->get_logger(), "Could not read status from mcu!");
      exit_node();
    }

    RCLCPP_INFO(this->get_logger(), "Sent hbeat: ID=0x%x, ctr=%d, csum=0x%x",
                (int)heartbeat_.id, (int)heartbeat_.counter, (int)heartbeat_.checksum);

    heartbeat_ctr_++;
    heartbeat_ctr_ = heartbeat_ctr_ % 256; // wrap around 255
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

  void exit_node(void)
  {
    RCLCPP_FATAL(this->get_logger(), "Exiting node!");
    rclcpp::shutdown();
    exit(EXIT_FAILURE); // Terminate the program
  }

  template <typename Type>
  uint8_t calculate_checksum(const Type &data)
  {
    const uint8_t *data_buffer = reinterpret_cast<const uint8_t *>(&data);
    size_t data_size = sizeof(data);

    uint8_t checksum = 0;

    for (size_t i = 0; i < data_size - 1; i++)
    {
      checksum ^= data_buffer[i];
    }

    return checksum;
  }

  template <typename TxType, typename RxType>
  int i2c_transaction(const TxType &tx_data, RxType *rx_data = nullptr)
  {
    const uint8_t *tx_buffer = reinterpret_cast<const uint8_t *>(&tx_data);
    size_t tx_size = sizeof(TxType);

    if (write(i2c_handle_, tx_buffer, tx_size) != static_cast<ssize_t>(tx_size))
    {
      return 1; // write error
    }

    // if we expect to read something back
    if (rx_data != nullptr)
    {
      // Convert the empty receive struct into a raw byte array so we can fill it
      uint8_t *rx_buffer = reinterpret_cast<uint8_t *>(rx_data);
      size_t rx_size = sizeof(RxType);

      if (read(i2c_handle_, rx_buffer, rx_size) != static_cast<ssize_t>(rx_size))
      {
        return 2; // read error
      }
    }

    return 0; // success
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