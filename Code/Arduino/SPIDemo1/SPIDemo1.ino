#include <SPI.h>
#include <SPISlave.h>

const uint data_len = 2;
const uint SPI_SPEED = 100 * 1000;  // hertz

SPISettings spisettings(SPI_SPEED, MSBFIRST, SPI_MODE0);

volatile bool recvBuffReady = false;
uint8_t recvBuff[data_len];                               // Use uint8_t for raw bytes
uint8_t sendBuff[data_len] = {25, 87};  // Example data to send back

void recvCallback(uint8_t *data, size_t len) {
  Serial.print("inside callback: \n");
  Serial.print(len); Serial.print("\t");
  Serial.println(*data);
  for (size_t i = 0; i < len; i++) {
    recvBuff[i] = data[i];
  }
  recvBuffReady = true;
}

void sentCallback() {
  SPISlave.setData(sendBuff, sizeof(sendBuff));
  // No need to do anything here, the data is pre-set in sendBuff
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  SPISlave.setSCK(2);
  SPISlave.setTX(3);
  SPISlave.setRX(4);
  SPISlave.setCS(5);

  sentCallback();

  SPISlave.onDataRecv(recvCallback);
  SPISlave.onDataSent(sentCallback);
  SPISlave.begin(spisettings);

  delay(3000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("S-INFO: SPISlave started");
}

void loop() {
  if (recvBuffReady) {
    Serial.print("S-RECV: ");
    for (int i = 0; i < data_len; i++) {
      // Serial.print("0x");
      Serial.print(recvBuff[i], DEC);  // Print as decimal
      Serial.print(",");
    }
    Serial.println("loop end");
    // Serial.println();
    // SPISlave.setData(sendBuff, data_len); // Send response
    recvBuffReady = false;
  }
}