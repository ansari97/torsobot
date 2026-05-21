#pragma once

#include <cstdint>
#include <limits> // for nan

static constexpr float SHARED_PI = 3.14159265358979323846f;

// i2c command variables
static constexpr uint8_t HEARTBEAT_ID = 0XAA;
static constexpr uint8_t MCU_CONTROLLER_CONFIG_ID = 0XBB;
static constexpr uint8_t INIT_DATA_ID = 0XCC;
static constexpr uint8_t STATE_DATA_ID = 0XDD;

// i2c structs
// hearbeat
struct __attribute__((packed)) Heartbeat
{
  uint8_t id = HEARTBEAT_ID;
  uint8_t counter;
  uint8_t checksum;
};

// config data
struct __attribute__((packed)) mcuConfigData
{
  uint8_t id = MCU_CONTROLLER_CONFIG_ID;
  float desired_torso_pitch = SHARED_PI; // vertically down
  float kp = 0.65;
  float ki = 0.0;
  float kd = 0.08;
  float wheel_max_torque = 3.5;   // N.m
  float control_max_integral = 4; // rad.s
  uint8_t controller = 1;
  uint8_t checksum;
};

// init data
struct __attribute__((packed)) InitData
{
  float torso_pitch_init = std::numeric_limits<float>::quiet_NaN();
  float wheel_rel_pos_init = std::numeric_limits<float>::quiet_NaN();
  uint8_t checksum;
};

// robot data
struct __attribute__((packed)) StateData
{
  float torso_pitch = std::numeric_limits<float>::quiet_NaN();
  float torso_pitch_rate = std::numeric_limits<float>::quiet_NaN();
  float wheel_pos = std::numeric_limits<float>::quiet_NaN();
  float wheel_vel = std::numeric_limits<float>::quiet_NaN();
  float encoder_ang = std::numeric_limits<float>::quiet_NaN();
  float encoder_speed = std::numeric_limits<float>::quiet_NaN();
  float wheel_cmd_torque = std::numeric_limits<float>::quiet_NaN();
  float wheel_actual_torque = std::numeric_limits<float>::quiet_NaN();
  int32_t encoder_steps = INT32_MAX;
  uint8_t motor_drv_mode = 255;
  uint8_t checksum;
};

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