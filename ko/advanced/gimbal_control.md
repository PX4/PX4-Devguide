# 짐벌 제어 설정

기체에 카메라(또는 다른 물건)를 달아 장착하는 짐벌을 제어하려면, 어떻게 제어할 지, PX4가 어떻게 명령을 내릴지 설정해야합니다. 여기서는 이 설정 방법을 설명합니다.

PX4에는 제각기 다른 입출력 방법을 갖는 일반적인 마운트/짐벌 컨트롤 드라이버가 있습니다.

- 입력은 (예를 들어, 임무 수행이나 측량할 경우) RC나 MAVLink 명령어를 통해 어떻게 짐벌을 제어할지 정의합니다.
- 출력은 MAVLink 명령 또는 비행 컨트롤러 AUX PWM 포트를 사용해 짐벌이 연결되는 방식을 정의합니다. 어떠한 입력 방법도 모든 출력을 구동 하는데 선택될 수 있으며, 입력 및 출력 모두 매개 변수를 통해 구성해야 합니다.

## 매개변수

[매개변수들](../advanced/parameter_reference.md#mount)은 설치 드라이버를 설정하는데 사용됩니다.

가장 중요한 부분은 입력([MNT_MODE_IN](../advanced/parameter_reference.md#MNT_MODE_IN)과 출력([MNT_MODE_OUT](../advanced/parameter_reference.md#MNT_MODE_OUT)) 모드입니다. 초기 값으로 입력은 활성화되어 있지 않고 드라이버도 작동하지 않습니다. 입력 모드를 선택하면, 재부팅이 되어 설치된 드라이버가 시작됩니다.

입력 모드가 `AUTO`로 설정되어 있으면, 가장 최근의 입력 수단을 기반으로 자동으로 모드가 바뀔 것입니다. MAVLink에서 RC로 전환하기 위해 스틱의 큰 움직임이 필요합니다.

## MAVLink 짐벌 (MNT_MODE_OUT=MAVLINK)

To enable a MAVLink gimbal, first set parameter [MNT_MODE_IN](../advanced/parameter_reference.md#MNT_MODE_IN) to `MAVLINK_DO_MOUNT` and [MNT_MODE_OUT](../advanced/parameter_reference.md#MNT_MODE_OUT) to `MAVLINK`.

The gimbal can be connected to *any free serial port* using the instructions in [MAVLink Peripherals (GCS/OSD/Companion)(https://docs.px4.io/master/en/peripherals/mavlink_peripherals.html#mavlink-peripherals-gcsosdcompanion) (also see [Serial Port Configuration](https://docs.px4.io/master/en/peripherals/serial_configuration.html#serial-port-configuration)).

A common configuration is to have a serial connection to the gimbal from the Flight Controller TELEM2 port (assuming TELEM2 is free). For this configuration you would set:

- [MAV_1_CONFIG](../advanced/parameter_reference.md#MAV_1_CONFIG) to **TELEM2** (if `MAV_1_CONFIG` is already used for a companion computer (say), use `MAV_2_CONFIG`).
- [MAV_1_MODE](../advanced/parameter_reference.md#MAV_1_MODE) to **NORMAL**
- [SER_TEL2_BAUD](../advanced/parameter_reference.md#SER_TEL2_BAUD) to manufacturer recommended baude rate.

This will enable the user to command the gimbal using [MAV_CMD_DO_MOUNT_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL) and [MAV_CMD_DO_MOUNT_CONFIGURE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE).

## Gimbal on Flight Controller (MNT_MODE_OUT=AUX)

The gimbal can be connected to the Flight controller AUX ports by setting the ouptut mode to `MNT_MODE_OUT=AUX`.

A mixer file is required to define the mapping for the output pins and the [mount mixer](https://github.com/PX4/PX4-Autopilot/blob/master/ROMFS/px4fmu_common/mixers/mount.aux.mix) is automatically selected (this overrides any AUX mixer provided by the airframe configuration).

The output assignment is as following:

- **AUX1**: Pitch
- **AUX2**: Roll
- **AUX3**: Yaw
- **AUX4**: Shutter/retract

### 믹서 구성 맞춤설정

> **Note** Read [Mixing and Actuators](../concept/mixing.md) for an explanation of how mixers work and the format of the mixer file.

The outputs can be customized by [creating a mixer file](../concept/system_startup.md#starting-a-custom-mixer) on the SD card named `etc/mixers/mount.aux.mix`.

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

The Typhoon H480 model comes with a preconfigured simulated gimbal.

To run it, use:

    make px4_sitl gazebo_typhoon_h480
    

To just test the mount driver on other models or simulators, make sure the driver runs (using `vmount start`), then configure its parameters.

## Testing

The driver provides a simple test command - it needs to be stopped first with `vmount stop`. The following describes testing in SITL, but the commands also work on a real device.

Start the simulation with (no parameter needs to be changed for that):

    make px4_sitl gazebo_typhoon_h480
    

Make sure it's armed, eg. with `commander takeoff`, then use the following command to control the gimbal (for example):

    vmount test yaw 30
    

Note that the simulated gimbal stabilizes itself, so if you send MAVLink commands, set the `stabilize` flags to `false`.

![Gazebo Gimbal Simulation](../../assets/simulation/gazebo/gimbal-simulation.png)