!REDIRECT "https://docs.px4.io/master/ko/advanced/gimbal_control.html"

# 짐벌 제어 설정

기체에 카메라(또는 다른 물건)를 달아 장착하는 짐벌을 제어하려면, 어떻게 제어할 지, PX4가 어떻게 명령을 내릴지 설정해야합니다. 여기서는 이 설정 방법을 설명합니다.

PX4에는 제각기 다른 입출력 방법을 갖는 일반적인 마운트/짐벌 컨트롤 드라이버가 있습니다.

- 입력은 (예를 들어, 임무 수행이나 측량할 경우) RC나 MAVLink 명령어를 통해 어떻게 짐벌을 제어할지 정의합니다.
- 출력은 MAVLink 명령 또는 비행 컨트롤러 AUX PWM 포트를 사용해 짐벌이 연결되는 방식을 정의합니다. 어떠한 입력 방법도 모든 출력을 구동 하는데 선택될 수 있으며, 입력 및 출력 모두 매개 변수를 통해 구성해야 합니다.

## 매개변수

[매개변수들](../advanced/parameter_reference.md#mount)은 설치 드라이버를 설정하는데 사용됩니다.

가장 중요한 부분은 입력([MNT_MODE_IN](../advanced/parameter_reference.md#MNT_MODE_IN)과 출력([MNT_MODE_OUT](../advanced/parameter_reference.md#MNT_MODE_OUT)) 모드입니다. 초기 값으로 입력은 활성화되어 있지 않고 드라이버도 작동하지 않습니다. 입력 모드를 선택하면, 재부팅이 되어 설치된 드라이버가 시작됩니다.

입력 모드가 `AUTO`로 설정되어 있으면, 가장 최근의 입력 수단을 기반으로 자동으로 모드가 바뀔 것입니다. MAVLink에서 RC로 전환하기 위해 스틱의 큰 움직임이 필요합니다.

## MAVLink에 연결된 짐벌 (MNT_MODE_OUT=MAVLINK)

MAVLink 짐벌을 사용하기 위해서는, 먼저 매개변수 [MNT_MODE_IN](../advanced/parameter_reference.md#MNT_MODE_IN) 을 `MAVLINK_DO_MOUNT`으로, [MNT_MODE_OUT](../advanced/parameter_reference.md#MNT_MODE_OUT)을 `MAVLINK`으로 설정하십시오.

짐벌은 [MAVLink Peripherals (GCS/OSD/Companion)(https://docs.px4.io/master/en/peripherals/mavlink_peripherals.html#mavlink-peripherals-gcsosdcompanion) 의 설명에 따라 *모든 사용가능한 직렬 포트*에 연결될 수 있습니다. (또한 [직렬 포트 설정](https://docs.px4.io/master/en/peripherals/serial_configuration.html#serial-port-configuration)을 보세요.

일반적인 구성은 비행 제어기 TELEM2 포트에서 짐벌에 직렬 연결하는 것입니다.(TELEM2가 사용가능한 경우) 이 구성의 경우 다음과 같이 설정합니다:

- [MAV_1_CONFIG](../advanced/parameter_reference.md#MAV_1_CONFIG) 을 **TELEM2**으로 설정. (만약 `MAV_1_CONFIG` 가 이미 보조컴퓨터에서 사용중인 경우, `MAV_2_CONFIG`을 사용).
- [MAV_1_MODE](../advanced/parameter_reference.md#MAV_1_MODE) 을 **NORMAL**로 설정.
- [SER_TEL2_BAUD](../advanced/parameter_reference.md#SER_TEL2_BAUD) 을 제조사 권장 보드 레이트(Baud rate)로 설정.

이렇게 하면 사용자가 [MAV_CMD_DO_MOUNT_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL) 과 [MAV_CMD_DO_MOUNT_CONFIGURE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE)를 이용해 짐벌에 명령할 수 있습니다.

## 비행컨트롤러에 연결된 짐벌

짐벌은 출력모드를 `MNT_MODE_OUT=AUX`로 설정해 비행 컨트롤러 AUX 포트에 연결할 수 있습니다.

믹서파일은 출력핀 설정을 위해 필요하며 [mount mixer](https://github.com/PX4/PX4-Autopilot/blob/master/ROMFS/px4fmu_common/mixers/mount.aux.mix)가 자동으로 선택됩니다.(이는 기체 설정에 의해 제공되는 AUX 믹서보다 우선으로 적용됩니다.)

출력 할당은 다음과 같습니다.

- **AUX1**: 상하 회전각(Pitch)
- **AUX2**: 좌우 회전각(Roll)
- **AUX3**: 방위 회전각(Yaw)
- **AUX4**: 셔터/원상복귀

### 믹서 구성 맞춤설정

> **주의** 믹서의 작동 및 믹서 파일의 형식에 대한 설명은 [혼합과 구동기](../concept/mixing.md)를 보시오.

출력은 [믹서 파일 만들기](../concept/system_startup.md#starting-a-custom-mixer)로 원하는 대로 변경이 가능하며, SD 카드의 `etc/mixers/mount.aux.mix`에 있습니다.

설치를 위한 기본 믹서 구성은 아래과 같습니다.

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

Typhoon H480 모델은 미리 설정모의된 짐벌과 함께 제공됩니다.

실행하려면 아래 명령어를 사용하십시오:

    make px4_sitl gazebo_typhoon_h480
    

다른 모델이나 시뮬레이터에 설치된 드라이버를 시험하려면, 설치된 드라이버가 `vmount start`를 사용해 작동하는지를 확인하고 난 후, 매개 변수들을 구성하십시오.

## 시험하기

이 드라이버는 간단한 시험 명령어를 제공합니다. 먼저 `vmount stop`으로 동작을 멈추어야합니다. 아래는 SITL에서의 시험 방법을 설명하지만, 이 명령어가 실제 장비에서도 작동합니다.

다음 명령으로 시작하십시오(매개 변수값을 바꿀 필요는 없습니다):

    make px4_sitl gazebo_typhoon_h480
    

시동 상태인지 확인하십시오, 예를 들면, `commander takeoff`를 입력하고 짐벌을 제어하기 위해 다음 명령어를 사용합니다:

    vmount test yaw 30
    

참고로 모의시험 진행시 짐벌은 스스로 안정화되므로, MAVLink 명령을 보낼 때는, `stabilize` 플래그 값을 `false`로 설정하십시오.

![Gazebo 짐벌 모의시험](../../assets/simulation/gazebo/gimbal-simulation.png)