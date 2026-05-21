//——————————————————————————————————————————————————————————————————————————————
// Demonstration of control and monitoring of 2 moteus controllers
// running on a CANBed FD from longan labs.
//  * https://mjbots.com/products/moteus-r4-11
//  * https://www.longan-labs.cc/1030009.html
//
// Controller ID 1 is moved through a sine wave pattern, while
// controller ID 2 just has a brake command sent.
// ——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <Moteus.h>

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CANBed FD board.
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2517_SCK = 2;  // SCK input of MCP2517
static const byte MCP2517_SDI = 3;  // SDI input of MCP2517
static const byte MCP2517_SDO = 4;  // SDO output of MCP2517

static const byte MCP2517_CS = 5;   // CS input of MCP2517
static const byte MCP2517_INT = 6;  // INT output of MCP2517

static uint32_t gNextSendMillis = 0;

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

// // technically should be the same as this block before the Moteus object initialization

// Moteus::Options options;
// options.id = 1;
// Moteus moteus1(can, optioons);

// Moteus moteus2(can, []() {
//   Moteus::Options options;
//   options.id = 2;
//   return options;
// }());

Moteus::PositionMode::Command position_cmd;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);

  // Let the world know we have begun!
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("started"));

  SPI.setMISO(MCP2517_SDO);  // same as MISO
  SPI.setCS(MCP2517_CS);
  SPI.setSCK(MCP2517_SCK);
  SPI.setMOSI(MCP2517_SDI);  // same as MOSI

  SPI.begin();

  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.
  // AAA: changed oscillator freq to 40MHz (same as MIKROE-2379)
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
    delay(1000);
  }

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus1.SetStop();
  // moteus2.SetStop();
  Serial.println(F("all stopped"));
}

uint16_t gLoopCount = 0;

void loop() {
  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }

  gNextSendMillis += 20;
  gLoopCount++;

  Moteus::PositionMode::Command cmd;

  // Moteus::CurrentMode::Command current_cmd;

  cmd.position = NaN;
  cmd.velocity = 0.2 * ::sin(time / 1000.0);

  // moteus1.SetCurrent(current_cmd);
  moteus1.SetPosition(cmd);
  // moteus2.SetBrake();
  if (gLoopCount % 5 != 0) {
    return;
  }

  // Only print our status every 5th cycle, so every 1s.

  Serial.print(F("time "));
  Serial.print(gNextSendMillis);

  auto print_moteus = [](const Moteus::Query::Result& query) {
    Serial.print(static_cast<int>(query.mode));
    Serial.print(F(" "));
    Serial.print(query.position);
    Serial.print(F("  velocity "));
    Serial.print(query.velocity);
    Serial.print(F("  temperature "));
    Serial.print(query.temperature);
  };

  print_moteus(moteus1.last_result().values);
  // Serial.print(F(" / "));
  // print_moteus(moteus2.last_result().values);
  Serial.println();
}
