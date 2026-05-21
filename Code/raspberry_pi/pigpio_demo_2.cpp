#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <cstring>

using namespace std;

const uint spiChannel = 0;         // CE_0 on SPI0
const uint spiSpeed = 1000 * 1000; // 1 MHz
const uint spiMode = 0;            // SPI mode

const int data_len = 8;   // bytes
char spiTxData[data_len]; // data transmit buffer
char spiRxData[data_len]; // data recived buffer

const int byte_len = 1;
char spiTxByte; // data transmit buffer
char spiRxByte; // data recived buffer

int spiReturnVal;

int main()
{
    // initialize pigpio
    int pi = gpioInitialise();
    if (pi < 0)
    {
        cerr << "Error initializing pigpio" << endl;
        usleep(100);
        return -1;
    };
    cout << "Initialized pigpio" << endl;

    // open SPI channel
    int spiHandle = spiOpen(spiChannel, spiSpeed, 0);
    if (spiHandle < 0)
    {
        cerr << "Error initializing pigpio SPI" << endl;
        usleep(100);
        return -1;
    };
    cout << "Initialized SPI" << endl;

    uint16_t iter = 0;
    char input_val[data_len];

    // // Send first byte to clear Pico's sendBuff
    // cout << "Sending dummy byte..." << endl;
    // cout << spiTxByte << endl;
    // spiReturnVal = spiXfer(spiHandle, &spiTxByte, &spiRxByte, byte_len);
    // cout << spiRxByte << endl;

    while (true)
    {
        memset(spiTxData, ' ', data_len);
        cout << "Enter a input string: ";
        cin >> input_val;

        sprintf(spiTxData, input_val);

        cout << spiTxData << endl;
        memset(spiRxData, 0, data_len);

        for (int i = 0; i < data_len; i++)
        {
            memset(&spiTxByte, spiTxData[i], byte_len);
            cout << "S: " << spiTxByte << endl;
            spiReturnVal = spiXfer(spiHandle, &spiTxByte, &spiRxByte, byte_len);
            memset(spiRxData + i, spiRxByte, byte_len);

            cout << "R (int): " << static_cast<int>(spiRxData[i]) << endl;
            cout << "R: " << spiRxData[i] << endl;
            memset(&spiRxByte, 0, byte_len);
            usleep(10);
        }
        // spiRxData
        cout << spiRxData << endl;
    }
}