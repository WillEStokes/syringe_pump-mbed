#include "debug.h"
#include "SyringePump.h"
#include "mbed.h"
#include "EthernetInterface.h"

SyringePump syringePump(
        PTD2, // mosi
        PTD3, // miso
        PTD1, // sclk
        PTC2, // ss
        PTC3, // dirPin
        PTD0, // stepPin
        //PTB21, // stepPin (blue LED for debugging)
        PTB18, // maxLimSwPin
        PTB19, // minLimSwPin
        PTC10, // statusLED (LED1) or green
        PTA1, // errorLED (LED2) or yellow
        LED1, // redLE
        PTB23, // stepperErrorPin
        PTA2, // stepperResetPin
        PTB2); // slaPin

int main(int, char**) {

    // Run
    syringePump.run();

}
