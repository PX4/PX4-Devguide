# 짐벌 제어 설정

If you want to control a gimbal with a camera (or any other payload) attached to the vehicle, you need to configure how you want to control it and how PX4 can command it. 여기서는 이 설정에 관해 설명합니다.

PX4는 다른 입력과 출력 방법을 갖는 일반적인 마운트/짐벌 제어 드라이버를 갖고 있습니다. 입력은 (예를 들어, 임무 수행이나 측량할 경우) RC나 MAVLink 명령어를 통해 어떻게 짐벌을 제어할지 정의합니다. 출력은 어떻게 이 짐벌이 연결되는지를 정의하는데, MAVLink 명령어를 지원하는 경우가 있고, 다른 PWM을 사용하는 경우가 있습니다. (PWM은 아래에 AUX 출력에서 설명합니다.) 모든 입력 방법은 어떤 방식으로든 출력될 수 있습니다. 두 방식 모두 매개변수를 통해 구성되어야 합니다.

## Parameters

[These parameters](../advanced/parameter_reference.md#mount) are used to setup the mount driver. The most important ones are the input (`MNT_MODE_IN`) and the output (`MNT_MODE_OUT`) mode. By default, the input is disabled and the driver does not run. After selecting the input mode, reboot the vehicle so that the mount driver starts.

If the input mode is set to `AUTO`, the mode will automatically be switched based on the latest input. To switch from mavlink to RC, a large stick motion is required.

## AUX output

If the output mode is set to `AUX`, a mixer file is required to define the mapping for the output pins and the [mount mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/mount.aux.mix) is automatically selected (overriding any aux mixer provided by the airframe configuration).

The output assignment is as following:

- **AUX1**: Pitch
- **AUX2**: Roll
- **AUX3**: Yaw
- **AUX4**: Shutter/retract

### Customizing the mixer configuration

> **Note** Read [Mixing and Actuators](../concept/mixing.md) for an explanation of how mixers work and the format of the mixer file.

The outputs can be customized by [creating a mixer file](../advanced/system_startup.md#starting-a-custom-mixer) on the SD card with name `etc/mixers/mount.aux.mix`.

A basic basic mixer configuration for a mount is shown below.

    # roll
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 0  10000  10000      0 -10000  10000
    
    # pitch
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 1  10000  10000      0 -10000  10000
    
    # yaw
    M: 1
    O:      10000  10000      0 -10000  10000
    S: 2 2  10000  10000      0 -10000  10000
    

## SITL

The Typhoon H480 model comes with a preconfigured simulated gimbal. To run it, use:

    make posix gazebo_typhoon_h480
    

To just test the mount driver on other models or simulators, make sure the driver runs, using `vmount start`, then configure its parameters.

## Testing

The driver provides a simple test command - it needs to be stopped first with `vmount stop`. The following describes testing in SITL, but the commands also work on a real device.

Start the simulation with (no parameter needs to be changed for that):

    make posix gazebo_typhoon_h480
    

Make sure it's armed, eg. with `commander takeoff`, then use for example

    vmount test yaw 30
    

to control the gimbal. Note that the simulated gimbal stabilizes itself, so if you send mavlink commands, set the `stabilize` flags to false.

![Gazebo Gimbal Simulation](../../assets/gazebo/gimbal-simulation.png)