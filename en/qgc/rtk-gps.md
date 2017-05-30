# RTK GPS Configuration

RTK (Real Time Kinematic) increases GPS accuracy to centimeter-level. It uses measurements of the phase of the signal's carrier wave, rather than the information content of the signal, and relies on a single reference station to provide real-time corrections, providing up to centimetre-level accuracy.

PX4 currently ONLY supports the single-frequency (L1) UBlox M8P based GNSS receivers for RTK.

### Working 

Two M8P GPS modules (see below for example setups) and a datalink is required to set up RTK with PX4. The unit on the ground (static position) is called the Base, and the in-air unit is called the Rover. The Base unit connects to QGroundControl and uses the datalink to the vehicle to stream RTCM corrections to it (using the MAVLink `GPS_RTCM_DATA` message). On the autopilot, the MAVLink packets are unpacked and sent to the GPS unit where they are processed to get the RTK solution.

The datalink should typically be able to handle an uplink rate of 300 bytes per second. See RTCM Messages below for more.

> **Note** : When using UBlox M8P modules **with the TX line from the flight controller connected to the GPS module's RX**, the PX4 GPS driver will automatically setup up the receiver module to send and receive the correct messages over the UART. This is also necessary to stream RTCM corrections to the GPS from the autopilot.

### RTCM Messages

QGroundControl configures the RTK base station to output the following RTCM3.2 frames :

1005 - Station coordinates XYZ for antenna reference point. (Base position).
1077 - Full GPS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)
1087 - Full GLONASS pseudo-ranges, carrier phases, Doppler and signal strength (high
resolution)
1127 -  Full BeiDou pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)
1097 - Full Galileo pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)

### Uplink datarate

The raw RTCM messages from the base are packed into a MAVLink `GPS_RTCM_DATA` message and sent over the datalink. The length of each MAVLink message is 182 bytes, and usually

The Base Position message (1005) is of length 22 bytes, while the others are all of variable length depending on the satellites visible and the number of signals from the satellite (only 1 for L1 units like M8P). Since at a given time, the maximum number of satellites visible from any single constellation is 12, the RTCM messages are around 120 bytes each. Therefore under real-world conditions, an uplink rate of 300 bps is sufficient  

## Drotek Tiny XXL Example 