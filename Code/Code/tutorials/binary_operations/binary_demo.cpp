#include <bitset>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace std;

double sensor_val;

const int arr_len = sizeof(sensor_val);
char demo_char[arr_len];

int main() {
  // should be empty
  cout << demo_char << endl;
  cout << sizeof(sensor_val) << endl;
  sensor_val = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  cout << sensor_val << endl;
  memcpy(demo_char, &sensor_val, sizeof(sensor_val));
//   for (int i = 0; i < arr_len; ++i) {
//     bitset<8> byte(demo_char[i]); // Create a bitset from each byte
//     cout << byte << " ";
//   }
//   cout << endl;

  double recovered_val;

  memcpy(&recovered_val, demo_char, sizeof(demo_char));
  cout << recovered_val << endl;

  return 1;
}
