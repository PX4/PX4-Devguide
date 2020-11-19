# 새 에어프레임 설정 추가

PX4에서는 에어프레임 시작점과 같은 설정값을 잘 포장한 에어프레임 설정을 활용합니다. 설정값들은 [ROMFS/px4fmu_common/init.d](https://github.com/PX4/PX4-Autopilot/tree/master/ROMFS/px4fmu_common/init.d)폴더에 저장된 [설정 파일](#config-file) 에 정의되어 있습니다. 설정 파일들은 [믹서 파일](#mixer-file)을 참조합니다. 이 파일은 시스템의 물리 설정을 기술하며, [ROMFS/px4fmu_common/mixers](https://github.com/PX4/PX4-Autopilot/tree/master/ROMFS/px4fmu_common/mixers)폴더에 저장되어 있습니다.

Adding a configuration is straightforward: create a new config file in the [init.d/airframes folder](https://github.com/PX4/PX4-Autopilot/tree/master/ROMFS/px4fmu_common/init.d/airframes) (prepend the filename with an unused autostart ID), add the name of your new airframe config file to the [CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/master/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt) in the relevant section, then [build and upload](../setup/building_px4.md) the software.

자체 설정을 만들고 싶지 않은 개발자는 대신 [개별 시스템 시작](../concept/system_startup.md) 페이지에서 자세하게 설명한 대로 microSD 카드에서 텍스트 파일로 이루어진 기존 설정을 약간 고칠 수 있습니다.

> **Note** To determine which parameters/values need to be set in the configuration file, you can first assign a generic airframe and tune the vehicle, and then use [`param show-for-airframe`](../middleware/modules_command.html#param) to list the parameters that changed.

## 설정 파일 개요

설정 파일과 믹서 파일의 설정 내용은 몇가지 메인 블록으로 이루어져있습니다:

* 에어프레임 문서([에어프레임 참조](../airframes/airframe_reference.md)와 *QGroundControl*에서 사용).
* [게인 조정](#tuning-gains)을 동반한 기체별 매개변수 설정.
* 시작해야 할 컨트롤러와 프로그램 (예: 멀티콥터, 고정익 컨트롤러, 착륙 감지자 등)
* 시스템 물리 설정(예: 비행체, 날개, 멀티콥터 등). 이 부분을 [믹서](../concept/mixing.md)라 칭합니다.

대부분 독립적인 측면이 있는데 많은 설정이 에어프레임의 동일한 물리 형체를 공유하고 동일한 프로그램을 시작하며, 게인 조정에 있어서만 다름을 의미합니다.

> **Note** New airframe files are only automatically added to the build system after a clean build (run `make clean`).

<a id="config-file"></a>

### Config File

A typical configuration file is shown below ([original file here](https://github.com/PX4/PX4-Autopilot/blob/master/ROMFS/px4fmu_common/init.d/airframes/3033_wingwing)).

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

The next section specifies vehicle-specific parameters, including [tuning gains](#tuning-gains):

```bash
sh /etc/init.d/rc.fw_defaults

if [ $AUTOCNF = yes ]
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

Set frame type ([MAV_TYPE](https://mavlink.io/en/messages/common.html#MAV_TYPE)):

```bash
# Configure this as plane
set MAV_TYPE 1
```

Set the [mixer](#mixer-file) to use:

```bash
# Set mixer
set MIXER wingwing
```

Configure PWM outputs (specify the outputs to drive/activate, and the levels).

```bash
# Provide ESC a constant 1000 us pulse
set PWM_OUT 4
set PWM_DISARMED 1000
```

> **Warning** If you want to reverse a channel, never do this on your RC transmitter or with e.g `RC1_REV`. The channels are only reversed when flying in manual mode, when you switch in an autopilot flight mode, the channels output will still be wrong (it only inverts your RC signal). Thus for a correct channel assignment change either your PWM signals with `PWM_MAIN_REV1` (e.g. for channel one) or change the signs of the output scaling in the corresponding mixer (see below).

<a id="mixer-file"></a>

### Mixer File

> **Note** First read [Concepts > Mixing](../concept/mixing.md). This provides background information required to interpret this mixer file.

A typical mixer file is shown below ([original file here](https://github.com/PX4/PX4-Autopilot/blob/master/ROMFS/px4fmu_common/mixers/wingwing.main.mix)). A mixer filename, in this case `wingwing.main.mix`, gives important information about the type of airframe (`wingwing`), the type of output (`.main` or `.aux`) and lastly that it is a mixer file (`.mix`).

The mixer file contains several blocks of code, each of which refers to one actuator or ESC. So if you have e.g. two servos and one ESC, the mixer file will contain three blocks of code.

> **Note** The plugs of the servos / motors go in the order of the mixers in this file.

So MAIN1 would be the left aileron, MAIN2 the right aileron, MAIN3 is empty (note the Z: zero mixer) and MAIN4 is throttle (to keep throttle on output 4 for common fixed wing configurations).

A mixer is encoded in normalized units from -10000 to 10000, corresponding to -1..+1.

    M: 2
    O:      10000  10000      0 -10000  10000
    S: 0 0  -6000  -6000      0 -10000  10000
    S: 0 1   6500   6500      0 -10000  10000
    

Where each number from left to right means:

* M: 두개의 제어 입력에 대해 스케일러가 둘 있음을 나타냅니다. 믹서가 받을 제어 입력의 수를 나타냅니다.
* O: 출력 계수(음의 *1 , 양의 *1), 오프셋(여기서는 0), 출력 범위(여기서는 -1..+1)를 나타냅니다.  
  * PWM 신호를 반전하려면 출력 계수를 바꾸어야 합니다: ```O:      -10000  -10000      0 -10000  10000```
  * 기본 계수를 지정할 경우 이 행은 완전히 생략할 수 있습니다(또는 생략해야 합니다): ```O:      10000  10000   0 -10000  10000```
* S: 첫 입력 계수를 나타냅니다. 제어 그룹 #0 (비행체 제어)와 처음 입력(roll - 좌우 회전각) 의 입력을 취합니다. 좌우 회전각 제어 입력의 0.6배 조정하며 부호를 반전합니다(스케일 단위에 따라 -0.6은 -6000이 됨). 오프셋을 반영하지 않으며, 전체 범위(-1..+1)로 출력합니다.
* S: 두번째 입력 계수를 나타냅니다. 제어 그룹 #0 (비행체 제어)와 두번째 입력(roll - 상하 회전각)의 입력값을 취합니다. 상하 회전각 제어 입력의 0.65배로 조정합니다. 오프셋을 반영하지 않으며 전체 범위(-1..+1)로 출력합니다.

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

For a new airframe belonging to an existing group, you don't need to do anything more than provide documentation in the airframe description located at [ROMFS/px4fmu_common/init.d](https://github.com/PX4/PX4-Autopilot/tree/master/ROMFS/px4fmu_common/init.d).

If the airframe is for a **new group** you additionally need to:

1. 분류에 해당하는 svg 이미지를 문서 저장소에 추가하십시오(이미지를 넣지 않으면 삽입 안내 이미지가 뜹니다): 
  * PX4 개발 안내서: [assets/airframes/types](https://github.com/PX4/Devguide/tree/master/assets/airframes/types)
  * PX4 사용 안내서: [assets/airframes/types](https://github.com/PX4/px4_user_guide/tree/master/assets/airframes/types)
2. Add a mapping between the new group name and image filename in the [srcparser.py](https://github.com/PX4/PX4-Autopilot/blob/master/Tools/px4airframes/srcparser.py) method `GetImageName()` (follow the pattern below): 
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

The following *PX4 User Guide* topics explain how to tune the parameters that will be specified in the config file:

* [멀티콥터 PID 조정 안내](https://docs.px4.io/master/en/advanced_config/pid_tuning_guide_multicopter.html)
* [고정익 PID 조정 안내](https://docs.px4.io/master/en/advanced_config/pid_tuning_guide_fixedwing.html)
* [수직 이착륙기 설정](https://docs.px4.io/master/en/config_vtol/)

## QGroundControl에 새 에어프레임 추가

To make a new airframe available for section in the *QGroundControl* [airframe configuration](https://docs.px4.io/master/en/config/airframe.html):

1. 빌드한 바이너리를 정리하고 다시 빌드하십시오(예: `make clean` 명령 수행 후 `make px4_fmu-v5_default` 실행)
2. QGC를 열어 다음과 같이 **사용자 정의 펌웨어 파일...**을 선택하십시오:
  
  ![QGC 플래시 개별 펌웨어](../../assets/gcs/qgc_flash_custom_firmware.png)
  
  플래싱할 **.px4** 펌웨어 파일을 선택하라는 요청을 받습니다(이 파일은 zip으로 압축한 JSON 파일이며, 에어프레임 메타데이터가 들어있습니다).

3. Navigate to the build folder and select the firmware file (e.g. **PX4-Autopilot/build/px4_fmu-v5_default/px4_fmu-v5_default.px4**).

4. **확인**을 눌러 펌웨어 플래싱을 시작하십시오.
5. *QGroundControl*을 다시 시작하십시오.

The new airframe will then be available for selection in *QGroundControl*.