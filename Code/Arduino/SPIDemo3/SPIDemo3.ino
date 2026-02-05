#include <SPI.h>
#include <SPISlave.h>

SPISettings spisettings(1000 * 1000, MSBFIRST, SPI_MODE0);

volatile bool recvBuffReady = false;
const int byte_len = 1;
char recvByte[byte_len];

const int data_len = 4;
char recvBuff[data_len];

int recvIdx = 0;
// char recvFirstByte;

// int sendcbs = -1;
char sendByte[byte_len];

char sendBuff[data_len];
char sendBuff1[data_len] = { '1', '1', '1', '\0' };
char sendBuff2[data_len] = { '0', '0', '0', '\0' };
char sendErr[data_len] = { 'e', 'e', 'e', '\0' };
char dummyByte[byte_len] = { 'd' };

int i = 0;

void recvCallback(uint8_t *data, size_t len) {
  memcpy(recvBuff + recvIdx, data, byte_len);
  recvIdx += byte_len;

  // memset(sendBuff, 0, sizeof(sendBuff));
  memset(sendByte, 0, sizeof(sendByte));

  if (recvBuff[0] == '1') {
    memcpy(sendByte, &sendBuff1[recvIdx], byte_len);
  } else if (recvBuff[0] == 'x') {
    memcpy(sendByte, &sendBuff2[recvIdx], byte_len);
  } else {
    memcpy(sendByte, &sendErr[recvIdx], byte_len);
  }
  
  SPISlave1.setData((uint8_t *)sendByte, sizeof(sendByte));

  if (recvIdx >= data_len) {
    recvBuffReady = true;
    recvIdx = 0;
  }
}


void sentCallback() {
  // memset(sendBuff, 0, sizeof(sendBuff));
  // memset(sendByte, 0, sizeof(sendByte));

  // if (recvIdx != -1) {
  //   if (recvBuff[0] == '1') {
  //     memcpy(sendBuff, sendBuff1, data_len);
  //   } else if (recvBuff[0] == 'x') {
  //     memcpy(sendBuff, sendBuff2, data_len);
  //   } else {
  //     memcpy(sendBuff, sendErr, data_len);
  //   }
  //   memcpy(sendByte, &sendBuff[recvIdx], sizeof(sendByte));
  // }

  // else {
  //   memcpy(sendByte, dummyByte, sizeof(sendByte));
  // }
  // SPISlave1.setData((uint8_t *)sendByte, sizeof(sendByte));
}

void setup() {
  Serial.begin(115200);

  SPISlave1.setSCK(10);
  SPISlave1.setTX(11);
  SPISlave1.setRX(12);
  SPISlave1.setCS(13);
  // sentCallback();
  SPISlave1.onDataRecv(recvCallback);
  SPISlave1.onDataSent(sentCallback);
  SPISlave1.begin(spisettings);
  delay(3000);
  Serial.println("S-INFO: SPISlave started");
}

void loop() {
  if (recvBuffReady) {
    Serial.print("Received: ");
    Serial.print(recvBuff);
    Serial.println();
    memset(recvBuff, 0, sizeof(recvBuff));  // clear the buffer.
    i = 0;
    recvBuffReady = false;
  }
}