#include <iostream>
#include <pigpio.h>
#include <unistd.h>

using namespace std;

int main()
{
    while (true)
    {
        cout << "Hello World" << endl;
        usleep(1000 * 1000);
    }

    return 0;
}
