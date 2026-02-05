#include <iostream>
// #include <pigpio.h>
// #include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cstring>

using namespace std;

const uint spiChannel = 0;
const uint spiSpeed = 1000 * 1000; // hertz

const int data_len = 1;
unsigned char spiData[data_len];

int spiReturnVal;

int main()
{
    int wiringPiSetup = wiringPiSetupPinType(WPI_PIN_BCM);
    while (wiringPiSetup != 0)
    {
        cout << "Error initializing wiringPi" << endl;
        delay(100);
    };

    cout << "Initialized wiringPi" << endl;

    int SPISetup = wiringPiSPISetup(spiChannel, spiSpeed);
    while (SPISetup < 0)
    {
        cout << "Error initializing wiringPi" << endl;
        delay(100);
    };

    cout << "Initialized SPI" << endl;

    while (true)
    {

        memset(spiData, '\0', sizeof(spiData));

        char num[data_len];
        cout << "Enter a number: ";
        cin >> num;

        sprintf((char *)spiData, num);

        for (int i = 0; i < sizeof(spiData); i++)
        {
            cout << spiData[i] << "\t";
        }
        cout << endl;
        // cout << static_cast<int>(spiData[0]) << endl;

        spiReturnVal = wiringPiSPIDataRW(spiChannel, spiData, data_len);
        // wiringPiSPIDataRW
        // cout << spiReturnVal << endl;

        for (int i = 0; i < sizeof(spiData); i++)
        {
            cout << spiData[i] << "\t";
        }
        cout << endl;

        if (spiReturnVal <= 0)
            cout << "SPI transfer error" << endl;
        else
        {
            cout << "SPI transfer successful" << endl;
            cout << spiData[0] << endl;
        }
    }

    return 0;
}
