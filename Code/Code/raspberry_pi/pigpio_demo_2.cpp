#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <cstring>
// #include <wiringPi.h>
// #include <wiringPiSPI.h>

using namespace std;

const uint spiChannel = 0;         // CE_0 on SPI0
const uint spiSpeed = 1000 * 1000; // 1 MHz
const uint spiMode = 0;            // SPI mode

const int data_len = 16;  // bytes
char spiTxData[data_len]; // data transmit buffer
char spiRxData[data_len]; // data recived buffer

const int byte_len = 1;
char spiTxByte[byte_len]; // data transmit buffer
char spiRxByte[byte_len]; // data recived buffer

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

    while (true)
    {
        memset(spiTxData, ' ', data_len);
        cout << "Enter a input string: ";
        cin >> input_val;

        sprintf(spiTxData, input_val);

        cout << spiTxData << endl;

        for (int i = 0; i < data_len; i++)
        {
            memset(spiTxByte, spiTxData[i], byte_len);
            cout << spiTxByte << endl;
            spiReturnVal = spiXfer(spiHandle, spiTxByte, spiRxByte, byte_len);
            memset(&spiRxData[i], *spiRxByte, byte_len);
            usleep(5);
        }

        cout << spiRxData << endl;
    }
}