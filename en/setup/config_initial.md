# Initial Configuration

Before starting to develop on PX4, the system should be configured initially with a default configuration to ensure the hardware is set up properly and is tested. The video below explains the setup process with [Pixhawk hardware](../flight_controller/pixhawk.md) and [QGroundControl](../qgc/README.md). A list of supported reference airframes is [here](../airframes/architecture.md).

> **Info** [Download the DAILY BUILD of QGroundControl](https://docs.qgroundcontrol.com/en/releases/daily_builds.html) and follow the video instructions below to set up your vehicle. See the [QGroundControl Tutorial](../qgc/README.md) for details on mission planning, flying and parameter setting.

A list of setup options is below the video.

{% youtube %}https://www.youtube.com/watch?v=91VGmdSlbo4&rel=0&vq=hd720{% endyoutube %}

## Radio Control Options

The PX4 flight stack does not mandate a radio control system. It also does not mandate the use of individual switches for selecting flight modes.

### Flying without Radio Control

All radio control setup checks can be disabled by setting the parameter `COM_RC_IN_MODE` to `1`. This will not allow manual flight, but e.g. flying in 

### Single Channel Mode Switch

Instead of using multiple switches, in this mode the system accepts a single channel as mode switch. This is explained in the [legacy wiki](https://pixhawk.org/peripherals/radio-control/opentx/single_channel_mode_switch).

