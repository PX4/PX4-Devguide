# 새 에어프레임 설정 추가

PX4 uses canned airframe configurations as starting point for airframes. The configurations are defined in [config files](#config-file) that are stored in the [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) folder. The config files reference [mixer files](#mixer-file) that describe the physical configuration of the system, and which are stored in the [ROMFS/px4fmu_common/mixers](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/mixers) folder.

Adding a configuration is straightforward: create a new config file in the [init.d folder](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) (prepend the filename with an unused autostart ID), then [build and upload](../setup/building_px4.md) the software.

Developers who do not want to create their own configuration can instead customize existing configurations using text files on the microSD card, as detailed on the [custom system startup](../concept/system_startup.md) page.

## 설정 파일 개요

The configuration in the config and mixer files consists of several main blocks:

* Airframe documentation (used in the [Airframes Reference](../airframes/airframe_reference.md) and *QGroundControl*).
* Vehicle-specific parameter settings, including [tuning gains](#tuning-gains).
* The controllers and apps it should start, e.g. multicopter or fixed wing controllers, land detectors etc.
* The physical configuration of the system (e.g. a plane, wing or multicopter). This is called a [mixer](../concept/mixing.md).

These aspects are mostly independent, which means that many configurations share the same physical layout of the airframe, start the same applications and differ most in their tuning gains.

> **Note** New airframe files are only automatically added to the build system after a clean build (run `make clean`).

### 설정 파일 {#config-file}

A typical configuration file is shown below ([original file here](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/airframes/3033_wingwing)).

The first section is the airframe documentation. This is used in the [Airframes Reference](../airframes/airframe_reference.md) and *QGroundControl*.

```bash
#!nsh
#
# @name Wing Wing (aka Z-84) Flying Wing
#
# @url https://docs.px4.io/master/en/framebuild_plane/wing_wing_z84.html
#
# @type Flying Wing
# @class Plane
#
# @output MAIN1 left aileron
# @output MAIN2 right aileron
# @output MAIN4 throttle
#
# @output AUX1 feed-through of RC AUX1 channel
# @output AUX2 feed-through of RC AUX2 channel
# @output AUX3 feed-through of RC AUX3 channel
#
# @maintainer Lorenz Meier <lorenz@px4.io>
#
```

다음 절에서는 [게인 조정](#tuning-gains)과 비행체별 매개변수를 지정합니다:

```bash
sh /etc/init.d/rc.fw_defaults

if [ $AUTOCNF == yes ]
then
  param set BAT_N_CELLS 2
  param set FW_AIRSPD_MAX 15
  param set FW_AIRSPD_MIN 10
  param set FW_AIRSPD_TRIM 13
  param set FW_R_TC 0.3
  param set FW_P_TC 0.3
  param set FW_L1_DAMPING 0.74
  param set FW_L1_PERIOD 16
  param set FW_LND_ANG 15
  param set FW_LND_FLALT 5
  param set FW_LND_HHDIST 15
  param set FW_LND_HVIRT 13
  param set FW_LND_TLALT 5
  param set FW_THR_LND_MAX 0
  param set FW_PR_FF 0.35
  param set FW_RR_FF 0.6
  param set FW_RR_P 0.04
fi
```

프레임 형식([MAV_TYPE](https://mavlink.io/en/messages/common.html#MAV_TYPE))을 설정하십시오:

```bash
# Configure this as plane
set MAV_TYPE 1
```

사용할 [믹서](#mixer-file)를 설정하십시오

```bash
# Set mixer
set MIXER wingwing
```

PWM 출력을 설정하십시오(제어/활성/레벨 출력을 지정하십시오).

```bash
# Provide ESC a constant 1000 us pulse
set PWM_OUT 4
set PWM_DISARMED 1000
```

> **Warning** If you want to reverse a channel, never do this on your RC transmitter or with e.g `RC1_REV`. The channels are only reversed when flying in manual mode, when you switch in an autopilot flight mode, the channels output will still be wrong (it only inverts your RC signal). Thus for a correct channel assignment change either your PWM signals with `PWM_MAIN_REV1` (e.g. for channel one) or change the signs of the output scaling in the corresponding mixer (see below).

### Mixer File {#mixer-file}

> **Note** First read [Concepts > Mixing](../concept/mixing.md). This provides background information required to interpret this mixer file.

A typical mixer file is shown below ([original file here](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/wingwing.main.mix)). A mixer filename, in this case `wingwing.main.mix`, gives important information about the type of airframe (`wingwing`), the type of output (`.main` or `.aux`) and lastly that it is a mixer file (`.mix`).

The mixer file contains several blocks of code, each of which refers to one actuator or ESC. So if you have e.g. two servos and one ESC, the mixer file will contain three blocks of code.

> **Note** The plugs of the servos / motors go in the order of the mixers in this file.

So MAIN1 would be the left aileron, MAIN2 the right aileron, MAIN3 is empty (note the Z: zero mixer) and MAIN4 is throttle (to keep throttle on output 4 for common fixed wing configurations).

A mixer is encoded in normalized units from -10000 to 10000, corresponding to -1..+1.

    M: 2
    O:      10000  10000      0 -10000  10000
    S: 0 0  -6000  -6000      0 -10000  10000
    S: 0 1   6500   6500      0 -10000  10000
    

왼편에서 오른편 방향으로 각 숫자의 의미는 다음과 같습니다:

* M: 두개의 제어 입력에 대해 스케일러가 둘 있음을 나타냅니다. 믹서가 받을 제어 입력의 수를 나타냅니다.
* O: 출력 계수(음의 *1 , 양의 *1), 오프셋(여기서는 0), 출력 범위(여기서는 -1..+1)를 나타냅니다.  
  * PWM 신호를 반전하려면 출력 계수를 바꾸어야 합니다: ```O:      -10000  -10000      0 -10000  10000```
  * 기본 계수를 지정할 경우 이 행은 완전히 생략할 수 있습니다(또는 생략해야 합니다): ```O:      10000  10000   0 -10000  10000```
* S: Indicates the first input scaler: It takes input from control group #0 (Flight Control) and the first input (roll). It scales the roll control input * 0.6 and reverts the sign (-0.6 becomes -6000 in scaled units). It applies no offset (0) and outputs to the full range (-1..+1)
* S: Indicates the second input scaler: It takes input from control group #0 (Flight Control) and the second input (pitch). \ It scales the pitch control input * 0.65. It applies no offset (0) and outputs to the full range (-1..+1)

> **Note** In short, the output of this mixer would be SERVO = ( (roll input \* -0.6 + 0) \* 1 + (pitch input \* 0.65 + 0) \* 1 ) \* 1 + 0

Behind the scenes, both scalers are added, which for a flying wing means the control surface takes maximum 60% deflection from roll and 65% deflection from pitch.

The complete mixer looks like this:

```bash
Delta-wing mixer for PX4FMU
===========================

Designed for Wing Wing Z-84

This file defines mixers suitable for controlling a delta wing aircraft using
PX4FMU. The configuration assumes the elevon servos are connected to PX4FMU
servo outputs 0 and 1 and the motor speed control to output 3. Output 2 is
assumed to be unused.

Inputs to the mixer come from channel group 0 (vehicle attitude), channels 0
(roll), 1 (pitch) and 3 (thrust).

See the README for more information on the scaler format.

Elevon mixers
-------------
Three scalers total (output, roll, pitch).

The scaling factor for roll inputs is adjusted to implement differential travel
for the elevons.

This first block of code is for Servo 0...

M: 2
O:      10000  10000      0 -10000  10000
S: 0 0  -6000  -6000      0 -10000  10000
S: 0 1   6500   6500      0 -10000  10000

And this is for Servo 1...

M: 2
O:      10000  10000      0 -10000  10000
S: 0 0  -6000  -6000      0 -10000  10000
S: 0 1  -6500  -6500      0 -10000  10000

Note that in principle, you could implement left/right wing asymmetric mixing, but in general the two blocks of code will be numerically equal, and just differ by the sign of the third line (S: 0 1), since to roll the plane, the two ailerons must move in OPPOSITE directions.
The signs of the second lines (S: 0 0) are indentical, since to pitch the plane, both servos need to move in the SAME direction.

Output 2
--------
This mixer is empty.

Z:

Motor speed mixer
-----------------
Two scalers total (output, thrust).

This mixer generates a full-range output (-1 to 1) from an input in the (0 - 1)
range.  Inputs below zero are treated as zero.

M: 1
O:      10000  10000      0 -10000  10000
S: 0 3      0  20000 -10000 -10000  10000

```

## 새 에어프레임 그룹 추가

Airframe "groups" are used to group similar airframes for selection in [QGroundControl](https://docs.qgroundcontrol.com/en/SetupView/Airframe.html) and in the *Airframe Reference* documentation ([PX4 DevGuide](../airframes/airframe_reference.md) and [PX4 UserGuide](https://docs.px4.io/master/en/airframes/airframe_reference.html)). Every group has a name, and an associated svg image which shows the common geometry, number of motors, and direction of motor rotation for the grouped airframes.

The airframe metadata files used by *QGroundControl* and the documentation source code are generated from the airframe description, via a script, using the build command: `make airframe_metadata`

For a new airframe belonging to an existing group, you don't need to do anything more than provide documentation in the airframe description located at [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d).

If the airframe is for a **new group** you additionally need to:

1. 여러 svg 이미지를 문서 저장소에 추가하십시오(이미지를 넣지 않으면 삽입 안내 이미지가 뜹니다): 
  * PX4 개발 안내서: [assets/airframes/types](https://github.com/PX4/Devguide/tree/master/assets/airframes/types)
  * PX4 사용 안내서: [assets/airframes/types](https://github.com/PX4/px4_user_guide/tree/master/assets/airframes/types)
2. 새 그룹 이름과 이미지 파일 이름간의 관계를 [srcparser.py](https://github.com/PX4/Firmware/blob/master/Tools/px4airframes/srcparser.py)의 `GetImageName()` 메서드에 추가하십시오 (다음 반복 규칙 참조): 
      def GetImageName(self):
           """
           Get parameter group image base name (w/o extension)
           """
           if (self.name == "Standard Plane"):
               return "Plane"
           elif (self.name == "Flying Wing"):
               return "FlyingWing"
            ...
      ...
           return "AirframeUnknown"

3. *QGroundControl*을 업데이트하십시오: 
  * 여러 svg 이미지를 [src/AutopilotPlugins/Common/images](https://github.com/mavlink/qgroundcontrol/tree/master/src/AutoPilotPlugins/Common/Images)에 추가하십시오
  * 다음 반복 규칙을 참고하여 svg 이미지 참조를 [qgcimages.qrc](https://github.com/mavlink/qgroundcontrol/blob/master/qgcimages.qrc) 파일에 추가하십시오: 
        <qresource prefix="/qmlimages">
            ...
            <file alias="Airframe/AirframeSimulation">src/AutoPilotPlugins/Common/Images/AirframeSimulation.svg</file>
            <file alias="Airframe/AirframeUnknown">src/AutoPilotPlugins/Common/Images/AirframeUnknown.svg</file>
            <file alias="Airframe/Boat">src/AutoPilotPlugins/Common/Images/Boat.svg</file>
            <file alias="Airframe/FlyingWing">src/AutoPilotPlugins/Common/Images/FlyingWing.svg</file>
            ... > 
    
    **Note** 에어프레임 메타데이터의 이름을 바꾸면 (**srcparser.py**를 업데이트한 후) 펌웨어에 자동으로 반영(include)되어야 합니다.

## 게인 조정

다음 *PX4 사용자 안내서*의 일부 주제에서는 설정 파일에 지정한 매개변수 값을 설정하는 방법을 설명합니다:

* [멀티콥터 PID 조정 안내](https://docs.px4.io/master/en/advanced_config/pid_tuning_guide_multicopter.html)
* [고정익 PID 조정 안내](https://docs.px4.io/master/en/advanced_config/pid_tuning_guide_fixedwing.html)
* [수직 이착륙기 설정](https://docs.px4.io/master/en/config_vtol/)

## QGroundControl에 새 에어프레임 추가

*QGroundControl* [에어프레임 설정](https://docs.px4.io/master/en/config/airframe.html)섹션에 새 에어프레임을 추가하려면:

1. 빌드한 바이너리를 정리하고 다시 빌드하십시오(예: `make clean` 명령 수행 후 `make px4_fmu-v5_default` 실행)
2. QGC를 열어 다음과 같이 **사용자 정의 펌웨어 파일...**을 선택하십시오:
  
  ![QGC 플래시 개별 펌웨어](../../assets/gcs/qgc_flash_custom_firmware.png)
  
  플래싱할 **.px4** 펌웨어 파일을 선택하라는 요청을 받습니다(이 파일은 zip으로 압축한 JSON 파일이며, 에어프레임 메타데이터가 들어있습니다).

3. 빌드 폴더를 둘러보고 펌웨어 파일을 선택하십시오(예: **Firmware/build/px4_fmu-v5_default/px4_fmu-v5_default.px4**).

4. **확인**을 눌러 펌웨어 플래싱을 시작하십시오.
5. *QGroundControl*을 다시 시작하십시오.

이 과정을 거치고 나면 *QGroundControl*의 섹션에 나타납니다.