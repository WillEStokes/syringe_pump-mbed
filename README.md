# Syringe Pump Controller for K64F with AMIS30543 Stepper Driver
This repository contains a Visual Studio Code project for programming a K64F board coupled with the AMIS30543 stepper driver board. This combination is used to control a stepper syringe pump.

## Features
- **TCP Communication:** The K64F board can receive commands via TCP.
- **Versatile Control:** You can control the syringe pump's flow, hardware configuration, pump state, and retrieve system & error information.
## Available Commands
The board responds to the following TCP commands:

1. `FID_GET_STATUS` - Retrieve the status of the syringe pump.
2. `FID_STOP_PUMP` - Halt the syringe pump.
3. `FID_START_PUMP` - Start the syringe pump.
4. `FID_SET_HARDWARE_CONFIG` - Set the hardware configuration.
5. `FID_SET_FLOW_CONFIG` - Define the flow configuration.
6. `FID_GET_HARDWARE_CONFIG` - Retrieve the current hardware configuration.
7. `FID_MAX_PULL` - Configure the maximum pull.
8. `FID_MAX_PUSH` - Set the maximum push configuration.
9. `FID_DISABLE_MOTOR_HOLD` - Disable motor hold.
10. `FID_GET_STEPDRV_ERROR` - Fetch stepper driver error information.
11. `FID_GET_FLOW_CONFIG` - Retrieve flow configuration details.
12. `FID_RESET_PUMP` - Reset the syringe pump.
13. `FID_GET_PUMP_ERROR` - Fetch any pump error information.
14. `FID_GET_SYS_INFO` - Retrieve system details.
15. `FID_IDENTIFY_ITSELF` - Have the system identify itself.
Each of these commands corresponds to a message handler function which processes the command and provides the necessary response.

## Message Communication
### Message Header
Messages have a header defined as:

`typedef struct {`<br />
`    uint8_t packetLength;`<br />
`    uint8_t fid;`<br />
`    uint8_t error;`<br />
`} __attribute__((__packed__)) MessageHeader;`<br />

- `packetLength`: Length of the entire packet.
- `fid`: Functional ID, denoting the command type.
- `error`: Error status.

### Message Receiver Function
Messages are received in a continuous loop, waiting for a header and then processing the relevant command through the handler functions. Only the STOP_PUMP and GET_STATUS commands can interrupt an ongoing pump action. Unsupported messages are returned with an error.

## Getting Started
To use this repository:

Clone the repository to your machine.
1. Open the project using Visual Studio Code.
2. Ensure you have the necessary extensions and tools for working with K64F board.
3. Compile and deploy to the K64F board.
## Contribution
Contributions are welcome! If you encounter any issues or have features in mind, feel free to open an issue or a pull request.
