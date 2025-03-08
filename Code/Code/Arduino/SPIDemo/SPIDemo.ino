#include <SPI.h>
#include <SPISlave.h>

const uint data_len = 4;
const uint SPI_SPEED = 500 * 1000;  // hertz

SPISettings spisettings(SPI_SPEED, MSBFIRST, SPI_MODE0);

volatile bool recvBuffReady = false;
char recvBuff[data_len] = "";
int recvIdx = 0;
void recvCallback(uint8_t *data, size_t len) {
  memcpy(recvBuff + recvIdx, data, len);
  recvIdx += len;
  if (recvIdx == sizeof(recvBuff)) {
    recvBuffReady = true;
    recvIdx = 0;
  }
}

int sendcbs = 0;
// Note that the buffer needs to be long lived, the SPISlave doesn't copy it.  So no local stack variables, only globals or heap(malloc/new) allocations.
char sendBuff[data_len];
void sentCallback() {
  memset(sendBuff, 0, sizeof(sendBuff));
  sprintf(sendBuff, "Slave to Master Xmission %d", sendcbs++);
  SPISlave.setData((uint8_t *)sendBuff, sizeof(sendBuff));
}

void setup() {

  Serial.begin(115200);

  SPISlave.setSCK(2);
  SPISlave.setTX(3);
  SPISlave.setRX(4);
  SPISlave.setCS(5);

  // Ensure we start with something to send...
  sentCallback();

  // Hook our callbacks into the slave
  SPISlave.onDataRecv(recvCallback);
  SPISlave.onDataSent(sentCallback);
  SPISlave.begin(spisettings);
  delay(3000);
  Serial.println("S-INFO: SPISlave started");
}

void loop() {
  if (recvBuffReady) {
    Serial.println("\nS-RECV: ");
    for (int i = 0; i < data_len; i++) {
      Serial.print(static_cast<int>(recvBuff[i]));
      Serial.print("\t");
    }
    recvBuffReady = false;
  }
}