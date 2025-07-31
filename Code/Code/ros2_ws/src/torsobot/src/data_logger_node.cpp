// writes data to a txt file

#include "rclcpp/rclcpp.hpp"
#include "torsobot_interfaces/msg/torsobot_data.hpp"
#include "torsobot_interfaces/msg/torsobot_state.hpp"

// For CSV output
#include <fstream>
#include <string>
#include <filesystem>

class DataLoggerNode : public rclcpp::Node
{
public:
  DataLoggerNode() : Node("data_logger_node")
  {
    // Get the current system time
    auto now_chrono = std::chrono::system_clock::now();
    // Convert to time_t for std::localtime
    std::time_t now_c = std::chrono::system_clock::to_time_t(now_chrono);
    // Convert to broken-down time structure
    std::tm *ltm = std::localtime(&now_c); // Note: std::localtime is not thread-safe, but usually fine for a single startup

    // Format the time into a string (e.g., "YYYY-MM-DD_HH-MM-SS")
    std::stringstream ss;
    ss << std::put_time(ltm, "%Y-%m-%d_%H-%M-%S"); // Year-Month-Day_Hour-Minute-Second
    std::string timestamp_str = ss.str();

    // directory name
    std::string directory_name_str = "/home/pi/torsobot/Code/Code/ros2_ws/data_logs";
    std::string file_name_str = "torsobot_data_csv_" + timestamp_str;
    std::string full_file_path_str = directory_name_str + file_name_str;

    // check if directory exists
    std::filesystem::path dir_path(directory_name_str);

    if (!std::filesystem::is_directory(dir_path))
    {
      // create directory
      RCLCPP_INFO(this->get_logger(), "Directory does not exist");
    }
    // create empty csv file

    //  Create subscriber
    // subscriber_ = this->create_subscription<torsobot_interfaces::msg::TorsobotData>(
    //     "/robot_data",
    //     10,
    //     std::bind(&DataLoggerNode::dataCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Data Logger Node started, recording to '%s'", full_file_path_str.c_str());
  }

private:
  rclcpp::Subscription<torsobot_interfaces::msg::TorsobotData>::SharedPtr subscriber_;

  // void dataCallback(torsobot_interfaces::msg::TorsobotData::SharedPtr msg)
  // {
  //   //
  // }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
