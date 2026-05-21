#include <SPI.h>
#include <SPISlave.h>

SPISettings spisettings(1000 * 1000, MSBFIRST, SPI_MODE0);

volatile bool recvBuffReady = false;
const int byte_len = 1;
char recvByte[byte_len];

const int data_bytes = 6;
const int data_len = data_bytes + 2;
char recvBuff[data_len];

int recvIdx = 0;
// char recvFirstByte;

char sendByte[byte_len];

char sendBuff[data_len];
char dummyByte = 'd';
char sendBuff1[data_len] = { dummyByte, '1', '1','1', '1','1', '1', '\0' };
char sendBuff2[data_len] = { dummyByte, '0', '0','0', '0','0', '0', '\0' };
char sendErr[data_len] = { dummyByte, 'e', 'e','e', 'e','e', 'e', '\0' };

int i = 0;

void recvCallback(uint8_t *data, size_t len) {
  memcpy(recvBuff + recvIdx, data, byte_len);
  memset(sendBuff, 0, sizeof(sendBuff));
  memset(sendByte, 0, sizeof(sendByte));

  if (recvBuff[0] == '1') {
    memcpy(sendBuff, sendBuff1, data_len);
  } else if (recvBuff[0] == 'x') {
    memcpy(sendBuff, sendBuff2, data_len);
  } else {
    memcpy(sendBuff, sendErr, data_len);
  }

  memcpy(sendByte, &sendBuff[recvIdx - 1], sizeof(sendByte));

  recvIdx += byte_len;
  if (recvIdx >= data_len) {
    recvBuffReady = true;
    recvIdx = 0;
  }
}

// bool sendFlag = 0;
// char sendByte[byte_len];

// char sendBuff[data_len - 1];
// char sendBuff1[data_len - 1] = { '1', '1', '\0' };
// char sendBuff2[data_len - 1] = { '0', '0', '\0' };
// char sendErr[data_len - 1] = { 'e', 'e', '\0' };
// char dummyByte = 'd';

// int i = 0;

void sentCallback() {
  // memset(sendBuff, 0, sizeof(sendBuff));
  // memset(sendByte, 0, sizeof(sendByte));

  // if (recvBuff[0] == '1') {
  //   memcpy(sendBuff, sendBuff1, data_len - 1);
  // } else if (recvBuff[0] == 'x') {
  //   memcpy(sendBuff, sendBuff2, data_len - 1);
  // } else {
  //   memcpy(sendBuff, sendErr, data_len - 1);
  // }

  // // if (recvIdx != 0) {
  // memcpy(sendByte, &sendBuff[recvIdx-1], sizeof(sendByte));
  // // } else {
  // // This caters to the fact that the recvIdx is reset at the end of the callback for the last byte received
  // // do nothing
  // // }

  SPISlave1.setData((uint8_t *)sendByte, sizeof(sendByte));
}

void setup() {
  Serial.begin(115200);

  SPISlave1.setSCK(10);
  SPISlave1.setTX(11);
  SPISlave1.setRX(12);
  SPISlave1.setCS(13);
  sendByte[0] = 'g';
  // SPISlave1.setData((uint8_t *)sendByte, sizeof(sendByte));
  sentCallback();
  // sendFlag = 1;
  // delay(3000);
  // Serial.print("sendFlag:");
  // Serial.println(sendFlag);
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
    Serial.print("\tSent: ");
    Serial.print(sendBuff);
    // Serial.print("sendBuff1: ");
    // Serial.print(sendBuff1);
    Serial.println();
    memset(recvBuff, 0, sizeof(recvBuff));  // clear the buffer.
    i = 0;
    recvBuffReady = false;
  }
}