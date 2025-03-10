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
const uint spiSpeed = 1000 * 1000; // hertz
const uint spiFlags = 0;           // SPI mode

const int data_len = 2;
char spiTxData[data_len];
char spiRxData[data_len];

// vector<char> txbuffer(data_len);

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

    while (true)
    {
        cout << "Running loop num: " << iter++ << endl;
        int spiHandle = spiOpen(spiChannel, spiSpeed, 0);
        memset(spiTxData, '\0', sizeof(spiTxData));
        memset(spiRxData, '\0', sizeof(spiRxData));

        // Set data values to send
        // int rand_no = rand() % 1000 + 100;
        // cout << (rand_no) << endl;

        char num[4];
        cout << "Enter a number: ";
        cin >> num;

        sprintf(spiTxData, "ha");

        for (int i = 0; i < sizeof(spiTxData); i++)
        {
            cout << spiTxData[i] << "\t";
        }
        cout << endl;
        cout << "size of buffer: " << sizeof(spiTxData) << endl;

        // int spiReturnVal = spiWrite(spiHandle, spiTxData, data_len);
        int spiReturnVal = spiXfer(spiHandle, spiTxData, spiRxData, data_len);
        cout << "spiReturnVal = " << spiReturnVal << endl;

        // // spiReturnVal = spiRead(spiHandle, spiRxData, data_len);
        cout << "Printing Rx data: " << endl;
        for (int i = 0; i < sizeof(spiRxData); i++)
        {
            // if (spiRxData[i] == '\0')
            // {
            //     cout << "emp" << "\t";
            // }
            // else
            cout << (spiRxData[i]) << "\t";
        }
        cout << endl;

        if (spiReturnVal != data_len)
        {
            cerr << "SPI transfer error" << endl;
        }
        else
        {
            cout << "SPI transfer successful" << endl;
            // cout << static_cast<int>(spiTxData[0]) << endl;
        }

        cout << "\n----------------" << endl;
        // iter++;
        usleep(2000 * 1000);

        spiClose(spiHandle);
    }

    spiClose(spiHandle);

    return 0;
}
