
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

// keep track of current time
uint period_start_us;

float angle;

// SPI lines for the CAN controller
#define MCP2517_SCK 10
#define MCP2517_MOSI 11
#define MCP2517_MISO 12
#define MCP2517_CS 13  // CS input of MCP2517
#define MCP2517_INT 9  // INT output of MCP2517

// driveshaft encoder
// must be consecutive pins on the rp2040
#define encoder_pinA 16
#define encoder_pinB 17

const float gear_ratio = (38.0 / 16.0) * (48.0 / 16.0);

uint16_t gLoopCount = 0;
static uint32_t gNextSendMillis = 0;

ACAN2517FD can(MCP2517_CS, SPI1, MCP2517_INT);

Moteus moteus(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(BNO08X_CS, OUTPUT);
  // pinMode(MCP2517_CS, OUTPUT);

  // digitalWrite(BNO08X_CS, HIGH);  // Disable both devices initially
  // digitalWrite(MCP2517_CS, HIGH);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens

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

  Serial.println(F("Adafruit BNO08x and BLDC motor driver test!"));

  Serial.println("here");
  SPI1.setSCK(MCP2517_SCK);
  SPI1.setMOSI(MCP2517_MOSI);
  SPI1.setMISO(MCP2517_MISO);
  SPI1.setCS(MCP2517_CS);

  Serial.println("here1");

  SPI1.begin();

  Serial.println("here2");

  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.
  // AAA: changed oscillator freq to 40MHz (same as MIKROE-2379 setting)
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

  while (errorCode != 0) {
    Serial.print(F("CAN error 0x"));
    Serial.println(errorCode, HEX);
    delay(10);
  }

  // To clear any faults the controllers may have, we start by sending
  // a stop command to the motor.
  moteus.SetStop();
  Serial.println(F("motors stopped"));

  Serial.println(F("Starting loop ..."));
  delay(5000);
}


void loop() {

  encoder.update();


  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  Moteus::PositionMode::Command cmd;

  // Moteus::CurrentMode::Command current_cmd;

  // cmd.position = NaN;
  // cmd.velocity = 1 / 6 * gear_ratio;  //1 cycle in 1 minute at the wheel

  cmd.position = NaN;
  cmd.velocity = 2 * ::sin(time / 1000.0);

  // moteus.SetCurrent(current_cmd);
  moteus.SetPosition(cmd);
  // moteus2.SetBrake();
  if (gLoopCount % 5 != 0) {
    return;
  }

  // Only print our status every 5th cycle, so every 1s.

  Serial.print(F("time "));
  Serial.print(gNextSendMillis);

  auto print_moteus = [](const Moteus::Query::Result& query) {
    Serial.print(static_cast<int>(query.mode));
    Serial.print(F("  position "));
    Serial.print(query.position);
    Serial.print(F("  velocity "));
    Serial.print(query.velocity);
  };

  signed_encoder_val = -static_cast<int32_t>(encoder.step);
  angle = static_cast<float>(signed_encoder_val) / ENCODER_CPR * 360;


  // print_moteus(moteus.last_result().values);
  // print_moteus(moteus2.last_result().values);
  Serial.print(F(" wheel angle (rad)"));
  Serial.print(angle);
  Serial.print(F(" wheel velocity (rad/s)"));
  Serial.print((encoder.speed / ENCODER_uSPR) * 2 * PI);
  Serial.println();

  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
}

