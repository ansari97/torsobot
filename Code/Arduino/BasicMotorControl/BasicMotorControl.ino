//——————————————————————————————————————————————————————————————————————————————
// Demonstration of control and monitoring of 2 moteus controllers
// running on a CANBed FD from longan labs.
//  * https://mjbots.com/products/moteus-r4-11
//  * https://www.longan-labs.cc/1030009.html
//
// Controller ID 1 is moved through a sine wave pattern, while
// controller ID 2 just has a brake command sent.
// ——————————————————————————————————————————————————————————————————————————————

#include <Arduino.h>
#include <SPI.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CANBed FD board.
//——————————————————————————————————————————————————————————————————————————————

#define MCP2517_SCK 10
#define MCP2517_MOSI 11
#define MCP2517_MISO 12
#define MCP2517_CS 13  // CS input of MCP2517
#define MCP2517_INT 9  // INT output of MCP2517

static uint32_t gNextSendMillis = 0;

float pos;
const float gear_ratio = (38.0 / 16.0) * (48.0 / 16.0);
float init_pos;

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can(MCP2517_CS, SPI1, MCP2517_INT);

Moteus moteus(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());
// Moteus moteus2(can, []() {
//   Moteus::Options options;
//   options.id = 2;
//   return options;
// }());

Moteus::PositionMode::Command position_cmd;
Moteus::PositionMode::Format position_fmt;
Moteus::CurrentMode::Command current_cmd;
Moteus::CurrentMode::Format current_fmt;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Let the world know we have begun!
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("started"));
  SPI1.setCS(MCP2517_CS);
  SPI1.setSCK(MCP2517_SCK);
  SPI1.setTX(MCP2517_MOSI);
  SPI1.setRX(MCP2517_MISO);

  SPI1.begin();

  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.
  ACAN2517FDSettings settings(
    ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll, DataBitRateFactor::x1);

  // The atmega32u4 on the CANbed has only a tiny amount of memory.
  // The ACAN2517FD driver needs custom settings so as to not exhaust
  // all of SRAM just with its buffers.
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;

  // const uint32_t errorCode = can.begin(settings, [] {
  //   can.isr();
  // });

  const uint32_t errorCode = can.begin(settings, [] {
    can.isr();
  });

  if (errorCode != 0) {
    Serial.print(F("CAN error 0x"));
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus.SetStop();
  // moteus2.SetStop();
  Serial.println(F("all stopped"));

  position_fmt.kp_scale = Moteus::kFloat;
  position_fmt.kd_scale = Moteus::kFloat;
  position_fmt.feedforward_torque = Moteus::kFloat;
  // position_fmt.velocity = Moteus::kIgnore;

  position_cmd.position = NaN;
  position_cmd.velocity = 0.0f;  // Not required as resolution is set to ignore
  position_cmd.kp_scale = 0.0f;
  position_cmd.kd_scale = 0.0f;

  position_cmd.maximum_torque = 0.01;
  position_cmd.feedforward_torque = 0;

  // **Write to the motor
  moteus.SetPosition(position_cmd, &position_fmt);
  init_pos = moteus.last_result().values.position;
}

uint16_t gLoopCount = 0;

void loop() {
  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  position_cmd.maximum_torque = 0.01;
  position_cmd.feedforward_torque = 0;

  // **Write to the motor
  moteus.SetPosition(position_cmd, &position_fmt);
  // moteus1.SetPosition(cmd);
  // moteus2.SetBrake();

  if (gLoopCount % 5 != 0) { return; }

  // Only print our status every 5th cycle, so every 1s.

  Serial.print(F("time "));
  Serial.print(gNextSendMillis);

  pos = moteus.last_result().values.position;

  Serial.print("\tGear Ratio:");
  Serial.print(gear_ratio);

  Serial.print("\tMotor Position: ");
  Serial.print(revsToRad(pos - init_pos));

  Serial.print("\tWheel Position: ");
  Serial.print(revsToRad((pos - init_pos) / gear_ratio));

  Serial.println();
}

float radToDeg(float rad) {
  return rad * 180 / PI;
}

// change from revolutions to radians
float revsToRad(float revs) {
  return revs * 2 * PI;
}
