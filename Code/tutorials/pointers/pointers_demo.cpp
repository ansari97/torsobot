#include <iostream>
#include <cstdlib>
using namespace std;

int main()
{
    int var = 9;
    int *ptr = &var;
    cout << "ptr addr: " << ptr << endl;
    cout << "ptr val: " << *ptr << endl;

    *ptr = 180;
    cout << "ptr addr: " << ptr << endl;
    cout << "ptr addr addr: " << &ptr << endl;

    return 0;
}