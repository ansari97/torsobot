#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // UNIX standard function definitions
#include <cstring>   // memset
#include <cerrno>    // Error number definitions
#include <gpiod.hpp> //for GPIO control

// --- Configuration ---
#define SERIAL_PORT "/dev/serial0" // Or "/dev/ttyAMA0"
#define BAUDRATE B921600

using namespace std::chrono_literals;

class SerialTestNode : public rclcpp::Node
{
public:
    SerialTestNode() : Node("serial_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "This is the serial test node...");

        RCLCPP_INFO(this->get_logger(), "Resetting the MCU...");
        this->resetMCU();
        RCLCPP_INFO(this->get_logger(), "Resetting complete!");

        if (!setup_serial_port(SERIAL_PORT, BAUDRATE))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port!");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Opened serial port!");

        heartbeat_timer_ = this->create_wall_timer(10ms, std::bind(&SerialTestNode::send_heartbeat, this));
    }

    ~SerialTestNode()
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "Serial port closed.");
        }
    }

private:
    // Configuration for the GPIO pin
    const std::string GPIO_CHIP_NAME_ = "gpiochip4"; // Verify with 'ls /dev/gpiochip*' on Pi5
    const unsigned int RPI_GPIO_FOR_PICO_RUN_ = 23;  // GPIO23

    gpiod::chip chip_;
    gpiod::line mcu_run_line_;

    int serial_fd_ = -1;

    uint32_t heartbeat_counter_ = 0;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // heartbeat struct
    struct __attribute__((packed)) HeartbeatPacket
    {
        const uint8_t SERIAL_HEADER_A_ = 0xAA;
        const uint8_t SERIAL_HEADER_B_ = 0xBB;
        const uint8_t ID = 0X00;
        uint8_t counter;
        uint8_t checksum;
    };

    void send_heartbeat(void)
    {
        HeartbeatPacket tx_heartbeat_packet_;

        tx_heartbeat_packet_.counter = heartbeat_counter_++; // assign first, increment later

        tx_heartbeat_packet_.checksum = calculate_checksum(reinterpret_cast<const uint8_t *>(&tx_heartbeat_packet_), sizeof(HeartbeatPacket) - 1);

        // Send Data
        int bytes_written = write(serial_fd_, &tx_heartbeat_packet_, sizeof(HeartbeatPacket));

        if (bytes_written < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial Write Error: %s", strerror(errno));
        }
        else
        {
            // Optional: Log every 10th packet so we don't spam the console
            if (heartbeat_counter_ % 10 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Sent Packet #%u", tx_heartbeat_packet_.counter);
            }
        }
    }

    // Reset pico using the run pin
    void resetMCU(void)
    {
        chip_.open(GPIO_CHIP_NAME_);
        mcu_run_line_ = chip_.get_line(RPI_GPIO_FOR_PICO_RUN_);

        gpiod::line_request request_config;
        request_config.consumer = "ros2_mcu_run_toggle";                     // Set the consumer name
        request_config.request_type = gpiod::line_request::DIRECTION_OUTPUT; // Set the direction

        mcu_run_line_.request(request_config, true);
        mcu_run_line_.set_value(false); // set to low
        usleep(100 * 1000);             // 100 miliseconds
        mcu_run_line_.set_value(true);  // reset state
        usleep(1 * 1000 * 1000);        // wait for 1 seconds for arduino loop to start before I2C
    }

    bool setup_serial_port(const char *port, speed_t baud)
    {
        serial_fd_ = open(port, O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0)
            return false;

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0)
            return false;

        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);

        // Standard Raw Mode Settings (8N1, No Flow Control)
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag = 0; // No canonical mode, no echo
        tty.c_oflag = 0; // No output processing
        tty.c_iflag = 0; // No input processing (software flow control off)

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
            return false;

        return true;
    }

    uint8_t calculate_checksum(const uint8_t *data, size_t len)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < len; i++)
        {
            sum ^= data[i];
        }
        return sum;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
