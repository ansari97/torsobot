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
#include "torsobot_interfaces/msg/data.hpp"

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
float IMU_val;
float mot_vel;

// I2C write cmd for IMU data
const int IMU_CMD = 0x01;
const int MOTOR_VAL_CMD = 0x2;

// Heartbeat variables
uint8_t HEARTBEAT_ID = 0xAA;
uint8_t heartbeat_ctr = 0;
uint8_t heartbeat_checksum = 0;

class I2CNode : public rclcpp::Node
{
public:
    I2CNode() : Node("i2c_manager_node")
    {
        // call the reset mcu function
        this->resetMCU();

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
        timer_ = this->create_wall_timer(10ms, std::bind(&I2CNode::timer_callback, this)); // Use std::bind
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<torsobot_interfaces::msg::Data>::SharedPtr publisher_;
    gpiod::chip chip_;
    gpiod::line mcu_run_line_;

    // timer callback for getting data on i2c
    void timer_callback(void)
    {
        // Read / request sensor data from pico
        int get_sensor_val_result1 = GetSensorValue(IMU_CMD, &IMU_val);
        int get_sensor_val_result2 = GetSensorValue(MOTOR_VAL_CMD, &mot_vel);
        int heartbeat_check = SendHeartbeat();
        if (get_sensor_val_result1 > 0 && get_sensor_val_result2 > 0 && heartbeat_check > 0)
        {
            auto message = torsobot_interfaces::msg::Data();
            message.torso_pitch = IMU_val;
            message.motor_vel = mot_vel;
            RCLCPP_INFO(this->get_logger(), "Publishing IMU pitch: '%f'", message.torso_pitch);
            RCLCPP_INFO(this->get_logger(), "Publishing motor velocity '%f'", message.motor_vel);
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

    //   Send heartbeat to pico
    int SendHeartbeat(void)
    {
        uint8_t message[3];
        message[0] = HEARTBEAT_ID;                 // always constant
        message[1] = heartbeat_ctr;                // incremented by 1
        message[2] = heartbeat_ctr ^ HEARTBEAT_ID; // checksum to guard against data corrurption during i2c communication

        // write heatrbeat to i2c
        if (write(i2c_handle, &message, sizeof(message)) != sizeof(message))
        {
            RCLCPP_ERROR(this->get_logger(), "I2C write failed! ");
            return -1; // Indicate error
        }

        RCLCPP_INFO(this->get_logger(), "Sent heartbeat: ID=0x%x, Counter=%d, Checksum=0x%x",
                    (int)message[0], (int)message[1], (int)message[2]);

        heartbeat_ctr++;
        heartbeat_ctr = heartbeat_ctr % 256; // wrap around 255
        return 1;
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2CNode>());
    rclcpp::shutdown();
    return 0;
}
