#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <vector>
#include <cstdlib>
// #include <wiringPi.h>
// #include <wiringPiSPI.h>

using namespace std;

const uint spiChannel = 0;        // CE_0 on SPI0
const uint spiSpeed = 500 * 1000; // hertz
const uint spiFlags = 0;          // SPI mode

const int data_len = 4;
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
        cout << "Running loop num: " << iter << endl;

        // Set data values to send
        cout << "Printing Tx data: " << endl;
        for (int i = 0; i < data_len; i++)
        {
            // random data between 0 and 256
            spiTxData[i] = static_cast<char>(rand() % 256);
            cout << static_cast<int>(spiTxData[i]) << "\t";
        }
        cout << endl;

        int spiReturnVal = spiXfer(spiHandle, spiTxData, spiRxData, data_len);
        cout << "spiReturnVal = " << spiReturnVal << endl;

        cout << "Printing Rx data: " << endl;
        for (int i = 0; i < data_len; i++)
        {
            cout << static_cast<int>(spiRxData[i]) << "\t";
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
        iter++;
        usleep(5000 * 1000);
    }

    spiClose(spiHandle);

    return 0;
}
