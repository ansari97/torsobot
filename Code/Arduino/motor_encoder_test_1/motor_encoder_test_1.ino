// Motor test with Pico 2W

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

#include "PicoEncoder.h"

PicoEncoder encoder;

uint32_t raw_encoder_val;
int32_t signed_encoder_val;

// Encoder variables
const int ENCODER_PPR = 2048;               // puses per rev
const int ENCODER_CPR = ENCODER_PPR * 4;    // counts per rev
const int ENCODER_uSPR = ENCODER_CPR * 64;  //usteps per rev
float angle = 0, prev_angle = 0;
float speed;

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CAN Controller board.
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2517_SCK = 10;  // SCK input of MCP2517
static const byte MCP2517_SDI = 11;  // SDI input of MCP2517
static const byte MCP2517_SDO = 12;  // SDO output of MCP2517

static const byte MCP2517_CS = 13;  // CS input of MCP2517
static const byte MCP2517_INT = 9;  // INT output of MCP2517

static uint32_t gNextSendMillis = 0;

// driveshaft encoder
// must be consecutive pins on the rp2040
#define encoder_pinA 16
#define encoder_pinB 17

const float gear_ratio = (38.0 / 16.0) * (48.0 / 16.0);

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can(MCP2517_CS, SPI1, MCP2517_INT);

Moteus moteus(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

Moteus::PositionMode::Command position_cmd;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  delay(5000);

  // Let the world know we have begun!
  Serial.begin(115200);
  // while (!Serial) {}
  Serial.println("started");

  SPI1.setCS(MCP2517_CS);
  SPI1.setTX(MCP2517_SDI);
  SPI1.setRX(MCP2517_SDO);
  SPI1.setSCK(MCP2517_SCK);

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

  const uint32_t errorCode = can.begin(settings, [] {
    can.isr();
  });

  if (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
    return;
  }

  // use the same pins for both the calibrated and non-calibrated encoders
  if (encoder.begin(encoder_pinA)) {
    // If begin() returns FALSE (Failure), we enter this block
    Serial.println("Encoder NOT found! System Halted.");
    while (1) {
      // Freeze here so we don't try to run a broken robot
      delay(100);
    }
  }
  Serial.println("Encoder found!");

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus.SetStop();
  Serial.println("all stopped");

  delay(5000);
}

uint16_t gLoopCount = 0;
uint16_t current_time = 0, prev_time = 0;

void loop() {

  current_time = millis();

  encoder.update();
  // encoder.autoCalibratePhases();
  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  Moteus::PositionMode::Command cmd;
  // cmd.position = NaN;
  // cmd.velocity = 2 * ::sin(time / 1000.0);
  cmd.position = NaN;
  cmd.velocity = 1;  // (2 * (::pow(PI, 2) / 10) * ::cos(2 * PI * (time/1000) / 10) * gear_ratio) / (2 * PI);  //1 cycle in 1 minute at the wheel

  // moteus.SetPosition(cmd);

  if (gLoopCount % 5 != 0) { return; }

  // Only print our status every 5th cycle, so every 1s.

  Serial.print(F("time "));
  Serial.print(gNextSendMillis);
  Serial.print("\t");

  auto print_moteus = [](const Moteus::Query::Result& query) {
    Serial.print("mode: ");
    Serial.print(static_cast<int>(query.mode));
    Serial.print("\tposition: ");
    Serial.print(query.position);
    Serial.print("\tvelocity: ");
    Serial.print(query.velocity);
  };

  // print_moteus(moteus.last_result().values);
  signed_encoder_val = -static_cast<int32_t>(encoder.step);
  angle = static_cast<float>(signed_encoder_val) / ENCODER_CPR * 2 * PI;

  speed = (angle - prev_angle) * 1000 / (current_time - prev_time);
  prev_angle = angle;
  prev_time = current_time;



  // print_moteus(moteus.last_result().values);
  // print_moteus(moteus2.last_result().values);
  Serial.print(F(" wheel steps: "));
  Serial.print(signed_encoder_val);
  Serial.print(F(" wheel angle (rad): "));
  Serial.print(angle);
  Serial.print(F(" wheel velocity (rad/s): "));
  Serial.print(speed, 10);  // / ENCODER_uSPR) * 2 * PI);
  // Serial.println();
  Serial.println();
}
