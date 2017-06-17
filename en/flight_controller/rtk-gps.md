# UBlox M8P RTK GPS Configuration

RTK (Real Time Kinematic) increases GPS accuracy to centimeter-level. It uses measurements of the phase of the signal's carrier wave, rather than the information content of the signal, and relies on a single reference station to provide real-time corrections, providing up to centimetre-level accuracy.

PX4 currently ONLY supports the single-frequency (L1) UBlox M8P based GNSS receivers for RTK.

### Working 

Two M8P GPS modules (see below for example setups) and a datalink is required to set up RTK with PX4. The unit on the ground (static position) is called the Base, and the in-air unit is called the Rover. The Base unit connects to QGroundControl and uses the datalink to the vehicle to stream RTCM corrections to it (using the MAVLink `GPS_RTCM_DATA` message). On the autopilot, the MAVLink packets are unpacked and sent to the GNSS unit where they are processed to get the RTK solution.

The datalink should typically be able to handle an uplink rate of 300 bytes per second. See the Uplink Datarate section below for more.

### Automatic Configuration

Both QGroundControl and the autopilot firmware share the same [PX4 GPS driver stack](https://github.com/PX4/GpsDrivers). In practice, this means that support for new protocols and/or messages only need to be added to one place.

The PX4 GPS stack automatically sets up the UBlox M8P modules to send and receive the correct messages over the UART or USB, depending on where the module is connected (to QGroundControl or the autopilot.) No configuration using U-Center is necessary.

Note : M8P-1 vs M8P-2

### RTCM Messages

QGroundControl configures the RTK base station to output the following RTCM3.2 frames :

**1005** - Station coordinates XYZ for antenna reference point. (Base position.)
**1077** - Full GPS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)
**1087** - Full GLONASS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)
**1127** -  Full BeiDou pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)
**1097** - Full Galileo pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.) **Only supported with M8P Firmware Version 3.01 and above. See section below on firmware updates.**
**1127** -  Full BeiDou pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)

**TODO : QGC doesn't configure the Galileo message yet -- needs an update**

### Uplink Datarate

The raw RTCM messages from the base are packed into a MAVLink `GPS_RTCM_DATA` message and sent over the datalink. The length of each MAVLink message is 182 bytes, and usually

The Base Position message (1005) is of length 22 bytes, while the others are all of variable length depending on the satellites visible and the number of signals from the satellite (only 1 for L1 units like M8P). Since at a given time, the _maximum_ number of satellites visible from any single constellation is 12, the RTCM messages are around 120 bytes each. Under real-world conditions, an uplink rate of 300 bps is sufficient.

HOWEVER - 



## Drotek Tiny XXL Example 

![](../../assets/drotek_rtk_base.jpg)

![](../../assets/drotek_rtk_rover.jpg)

## HEX/ProfiCNC Here+ Example


