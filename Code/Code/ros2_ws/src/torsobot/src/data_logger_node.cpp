// writes data to a txt file
// copies parameters file

#include "rclcpp/rclcpp.hpp"
#include "torsobot_interfaces/msg/torsobot_data.hpp"
#include "torsobot_interfaces/msg/torsobot_state.hpp"

#include "torsobot_interfaces/srv/request_control_params.hpp"

#include <json.hpp> // For JSON output
#include <fstream>  // For CSV output and file system operations
#include <string>
#include <chrono>     // For std::chrono
#include <ctime>      // For std::localtime, std::time_t, std::tm
#include <iomanip>    // For std::put_time
#include <sstream>    // For std::stringstream
#include <functional> // For std::bind and std::placeholders
#include <filesystem> // for copying YAML files
#include <cstdlib>    // For getenv
#include <cmath>      // for isnan()
// #include <rosbag2_cpp/writer.hpp>

// for parsing metadata yaml file
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

// Define the CSV header row based on your message fields
const std::string CSV_HEADER = "timestamp,torso_pitch,torso_pitch_rate,wheel_pos,wheel_vel,wheel_torque,wheel_cmd_torque,mot_drv_mode,mot_pos,mot_vel";

// // parameters
// volatile float desired_torso_pitch;
// volatile float mot_max_torque, control_max_integral;
// volatile float kp, ki, kd;

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
    std::string csv_filename_str = "csv_" + timestamp_str + ".csv";
    std::string csv_full_path_str = directory_name_str + "/" + csv_filename_str;

    std::string metadata_filename_str = "metadata_" + timestamp_str + ".yaml";
    metadata_full_path_str_ = directory_name_str + "/" + metadata_filename_str;

    std::string param_source_file = "/home/pi/torsobot/Code/Code/ros2_ws/src/torsobot/config/params.yaml";

    // read metadata yaml file
    YAML::Node param_yaml_file = YAML::LoadFile(param_source_file);
    log_data_ = (param_yaml_file["/**"]["ros__parameters"]["log_data"].as<bool>());
    // if datalogging is not required, do not copy metadata file and do not make a csv and subscriber

    if (log_data_)
    {
      // check if directory exists
      std::filesystem::path dir_path(directory_name_str);
      if (!std::filesystem::is_directory(dir_path))
      {
        // create directory
        RCLCPP_INFO(this->get_logger(), "Directory does not exist. Creating directory...");

        // create directory
        try
        {
          std::filesystem::create_directories(dir_path);
        }
        catch (const std::filesystem::filesystem_error &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to create directory. Error: %s", e.what());
          // std::cerr << e.what() << '\n';
        }

        RCLCPP_INFO(this->get_logger(), "Directory created!");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Directory already exists!");
      }

      // copy metdata file
      try
      {
        std::filesystem::copy(param_source_file, metadata_full_path_str_, std::filesystem::copy_options::overwrite_existing);
        RCLCPP_INFO(this->get_logger(), "Parameter file copied successfully from %s to %s", param_source_file.c_str(), metadata_full_path_str_.c_str());

        // read the log_data_ parameter from the xml file
      }
      catch (const std::filesystem::filesystem_error &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Parameter file copy from %s to %s failed!", param_source_file.c_str(), metadata_full_path_str_.c_str());
      }

      // create empty csv file
      output_file_.open(csv_full_path_str);

      // Check if the file was opened successfully
      if (output_file_.is_open())
      {
        RCLCPP_INFO(this->get_logger(), "Successfully opened data file: '%s'", csv_full_path_str.c_str());
        // Write the header row
        output_file_ << CSV_HEADER << std::endl;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open data file: '%s'", csv_full_path_str.c_str());
        return; // Exit constructor if we can't open the file
      }

      // Create subscriber
      subscriber_ = this->create_subscription<torsobot_interfaces::msg::TorsobotData>(
          "torsobot_data",
          10,
          std::bind(&DataLoggerNode::dataCallback, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "Data Logger Node started, recording to '%s'", csv_full_path_str.c_str());
    }

    else
    {
      RCLCPP_INFO(this->get_logger(), "Not logging data...");
    }
  }

private:
  rclcpp::Client<torsobot_interfaces::srv::RequestControlParams>::SharedPtr service_client_;
  rclcpp::Subscription<torsobot_interfaces::msg::TorsobotData>::SharedPtr subscriber_;
  // rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream output_file_;
  std::string metadata_full_path_str_;
  YAML::Node metadata_yaml_file_;
  bool copied_init_values_ = false;
  bool log_data_;

  void dataCallback(const torsobot_interfaces::msg::TorsobotData::SharedPtr msg)
  {
    // Check if the file is still open before trying to write
    if (output_file_.is_open())
    {
      // Write the timestamp in nanoseconds
      output_file_ << this->now().nanoseconds() << ",";
      // Write the data from the message, separated by commas
      output_file_ << msg->torsobot_state.torso_pitch << "," << msg->torsobot_state.torso_pitch_rate << "," << msg->torsobot_state.wheel_pos << "," << msg->torsobot_state.wheel_vel << "," << msg->wheel_torque << "," << msg->wheel_cmd_torque << "," << static_cast<int>(msg->mot_drv_mode) << "," << msg->mot_pos << "," << msg->mot_vel;
      // Add a newline character to finish the row
      output_file_ << "\n";
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "File is not open, cannot write data.");
    }

    if (log_data_ && !copied_init_values_)
    {
      if (!(std::isnan(msg->torso_pitch_init) || std::isnan(msg->mot_pos_init)))
      {
        // Copy init position values for the torso and the motor into the new xml file.
        metadata_yaml_file_ = YAML::LoadFile(metadata_full_path_str_);
        metadata_yaml_file_["trial_values"]["torso_pitch_init"] = msg->torso_pitch_init;
        metadata_yaml_file_["trial_values"]["mot_pos_init"] = msg->mot_pos_init;

        std::ofstream fout(metadata_full_path_str_);
        fout << metadata_yaml_file_;

        copied_init_values_ = true;
      }
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
