#include "debug.h"
#include "EZOSensors.h"
#include "mbed.h"
#include "EthernetInterface.h"
#include "EZO.h"

EZOSensors ezosensors(
    LED1,
    D9); // green LED

int main(int, char**) {
    // Run
    ezosensors.run();

}

