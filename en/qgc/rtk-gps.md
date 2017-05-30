# RTK GPS Configuration

RTK (Real Time Kinematic) increases GPS accuracy to centimeter-level. It uses measurements of the phase of the signal's carrier wave, rather than the information content of the signal, and relies on a single reference station to provide real-time corrections, providing up to centimetre-level accuracy.

PX4 currently supports the single-frequency (L1) UBlox M8P based GNSS receivers for RTK. 

### Working 

2 M8P GPS modules (see below for example setups) and a datalink is required to set up RTK with PX4. The unit on the ground (static position) is called the Base, and the in-air unit is called the Rover. The Base unit connects to QGroundControl and uses the datalink to the vehicle to stream RTCM corrections to it using MAVLink (the ``GPS_RTCM_DATA` message is used). On the autopilot, the RTCM packets are unpacked

> **Note** : When using UBlox M8P modules **with the TX line from the flight controller connected to the GPS module's RX**, the PX4 GPS driver will automatically setup up the receiver module to send and receive the correct messages over the UART.

### RTCM Messages

PX4 configures the RTK base 

1005 - 
1077 - 
1087 -
1127 - 


## Drotek Tiny XXL Example 