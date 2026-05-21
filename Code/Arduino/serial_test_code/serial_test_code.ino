#include <Arduino.h>

#define UART_BAUD 921600

// heartbeat struct
struct __attribute__((packed)) HeartbeatPacket {
  uint8_t SERIAL_HEADER_A_;
  uint8_t SERIAL_HEADER_B_;
  uint8_t ID;
  uint8_t counter;
  uint8_t checksum;
} rx_heartbeat_packet_;

volatile uint8_t heartbeat_ctr;
volatile uint8_t heartbeat_last_ctr;

volatile bool is_heartbeat_valid;
volatile bool is_heartbeat_ctr_valid;

volatile uint64_t serial_ctr = 0;

uint64_t heartbeat_timer;

uint8_t calculate_checksum(const uint8_t *data, size_t len);


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial1 is the Hardware UART (GP0/GP1 usually)
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(UART_BAUD);

  // USB Serial for debugging (Optional)
  Serial.begin(115200);

  uint64_t t = millis();  //current time
  while (!Serial && millis() - t < 1000) {
    delay(1);
  }

  Serial.println("Starting code...");
}

void loop() {
  if (Serial1.available() >= sizeof(HeartbeatPacket)) {

    // 2. Peek at the first byte. Is it our Header 1 (0xAA)?
    if (Serial1.peek() != 0XAA) {
      // GARBAGE DATA: Read 1 byte to discard it and try again next loop
      Serial1.read();
      return;
    }

    // 3. We found 0xAA! Now read the whole packet
    // (We know the bytes are there because we checked available() above)
    Serial1.readBytes((uint8_t *)&rx_heartbeat_packet_, sizeof(HeartbeatPacket));

    // 4. Validate Header 2
    if (rx_heartbeat_packet_.SERIAL_HEADER_B_ != 0xBB) {
      // Bad header 2. Packet is corrupt or we are misaligned.
      return;
    }

    // 5. Validate Checksum
    // Calculate sum of everything EXCEPT the last byte (checksum itself)
    uint8_t heartbeat_expected_checksum = calculate_checksum((uint8_t *)&rx_heartbeat_packet_, sizeof(HeartbeatPacket) - 1);


    if (heartbeat_expected_checksum == rx_heartbeat_packet_.checksum) {
      if (serial_ctr++ == 0) heartbeat_last_ctr = heartbeat_ctr;

      is_heartbeat_valid = true;

      if (serial_ctr != 0) {
        if (rx_heartbeat_packet_.counter == (heartbeat_last_ctr + 1) % 256) {
          is_heartbeat_ctr_valid = true;
          heartbeat_last_ctr = rx_heartbeat_packet_.counter;
          heartbeat_timer = millis();

          if (serial_ctr % 10 == 0) {
            Serial.println(rx_heartbeat_packet_.counter);
          }
        }
      }
    } else {
      is_heartbeat_valid = false;
      Serial.println("checksum error!");
    }
  }
}

uint8_t calculate_checksum(const uint8_t *data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum ^= data[i];
  }
  return sum;
}
