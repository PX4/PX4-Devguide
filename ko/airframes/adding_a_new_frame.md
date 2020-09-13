# 새 에어프레임 설정 추가

PX4에서는 에어프레임 시작점과 같은 설정값을 잘 포장한 에어프레임 설정을 활용합니다. [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) 폴더에 저장한 [설정 파일 ](#config-file)에 설정을 정의합니다. 설정 파일은 시스템의 물리 설정을 기술한 [믹서 파일](#mixer-file)을 참조하며 이 파일은 [ROMFS/px4fmu_common/mixers](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/mixers) 폴더에 들어있습니다.

설정 추가는 굉장히 간단합니다: [init.d 폴더](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) (파일 이름 앞에 사용하지 않는 자동 시작 ID를 붙임)에 새 설정 파일을 넣고, 프로그램을 [빌드 후 업로드](../setup/building_px4.md) 합니다.

자체 설정을 만들고 싶지 않은 개발자는 대신 [개별 시스템 시작](../concept/system_startup.md) 페이지에서 자세하게 설명한 대로 microSD 카드에서 텍스트 파일로 이루어진 기존 설정을 약간 고칠 수 있습니다.

## 설정 파일 개요

설정 파일과 믹서 파일의 설정 내용은 몇가지 메인 블록으로 이루어져있습니다:

* 에어프레임 문서([에어프레임 참조](../airframes/airframe_reference.md)와 *QGroundControl*에서 사용).
* [게인 조정](#tuning-gains)을 동반한 기체별 매개변수 설정.
* 시작해야 할 컨트롤러와 프로그램 (예: 멀티콥터, 고정익 컨트롤러, 착륙 감지자 등)
* 시스템 물리 설정(예: 비행체, 날개, 멀티콥터 등). 이 부분을 [믹서](../concept/mixing.md)라 칭합니다.

대부분 독립적인 측면이 있는데 많은 설정이 에어프레임의 동일한 물리 형체를 공유하고 동일한 프로그램을 시작하며, 게인 조정에 있어서만 다름을 의미합니다.

> **Note** 새 에어프레임 파일은 빌드를 정리한 후에만 빌드 시스템에 자동으로 추가합니다(`make clean` 실행).

### 설정 파일 {#config-file}

보통 설정 파일은 아래와 같습니다([원본 파일은 여기에 있습니다](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/airframes/3033_wingwing)).

처음 부분은 에어프레임 문서입니다. [에어프레임 참조](../airframes/airframe_reference.md)와 *QGroundControl*에서 활용합니다.

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

다음 절에서는 [게인 조정](#tuning-gains)과 개체별 매개변수를 지정합니다:

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

> **Warning** 채널을 역순으로 보려면 RC 송수신기 또는 `RC1_REV`와 같은 매크로 매개변수에 이 작업을 수행하지 마십시오. 수동 모드로 비행체를 날릴 경우에만 채널을 반전합니다. 자동 비행 모드로 전환하면 채널 출력이 잘못될 수 있습니다(리모콘 신호만 반전할 수 있음). 따라서 올바른 채널 할당을 수행하려면 (예: 채널 하나에 대해) PWM 시그널 `PWM_MAIN_REV1`으로 PWM 시그널을 바꾸거나 믹서와 관련된 출력 계수 부호만 바꾸십시오(아래 참고).

### 믹서 파일 {#mixer-file}

> **Note** 우선 [개념 > 믹싱](../concept/mixing.md)을 읽어보십시오. 이 문서에서는 믹서 파일을 이해하는데 필요한 배경 지식을 전달합니다.

보통 믹서 파일은 아래와 같습니다([원본 파일은 여기에 있습니다](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/wingwing.main.mix)). 이 경우 믹서 파일 이름은 `wingwing.main.mix`이며, 중요한 에어프레임 형식(`wingwing`), 출력 형식(`.main` 또는 `.aux`), 믹서 파일을 의미하는 확장자(`.mix`)정보를 바로 전달해줍니다. 

믹서 파일에는 여러 코드 블록이 들어있으며, 각 코드 블록은 액츄에이터 또는 전동 변속기 하나를 참조합니다. 따라서 서보 모터 둘과 전동 변속기 유닛 하나를 붙였다면, 믹서 파일은 세개의 코드 블록을 가집니다.

> **Note** 서보 / 모터는 이 파일에서 언급한 순서대로 연결했습니다.

따라서 MAIN1은 좌측 보조익, MAIN2는 우측 보조익, MAIN3은 빈 상태(믹서가 없을때 Z: 로 표기함), MAIN4는 추진기입니다(일반 고정익 설정에서 4번 출력을 추진기로 둠).

믹서는 -10000부터 10000까지 정규화 된 값으로 인코딩하며, 이 값은 -1..+1에 해당합니다.

    M: 2
    O:      10000  10000      0 -10000  10000
    S: 0 0  -6000  -6000      0 -10000  10000
    S: 0 1   6500   6500      0 -10000  10000
    

왼편에서 오른편 방향으로 각 숫자의 의미는 다음과 같습니다:

* M: 두개의 제어 입력에 대해 스케일러가 둘 있음을 나타냅니다. 믹서가 받을 제어 입력의 수를 나타냅니다.
* O: 출력 계수(음의 *1 , 양의 *1), 오프셋(여기서는 0), 출력 범위(여기서는 -1..+1)를 나타냅니다.  
  * PWM 신호를 반전하려면 출력 계수를 바꾸어야 합니다: ```O:      -10000  -10000      0 -10000  10000```
  * 기본 계수를 지정할 경우 이 행은 완전히 생략할 수 있습니다(또는 생략해야 합니다): ```O:      10000  10000   0 -10000  10000```
* S: 첫 입력 계수를 나타냅니다. 제어 그룹 #0 (비행체 제어)와 처음 입력(roll - 좌우 회전각) 의 입력을 취합니다. 좌우 회전각 제어 입력의 0.6배 조정하며 부호를 반전합니다(스케일 단위에 따라 -0.6은 -6000이 됨). 오프셋을 반영하지 않으며, 전체 범위(-1..+1)로 출력합니다.
* S: 두번째 입력 계수를 나타냅니다. 제어 그룹 #0 (비행체 제어)와 두번째 입력(roll - 상하 회전각)의 입력값을 취합니다. 상하 회전각 제어 입력의 0.65배로 조정합니다. 오프셋을 반영하지 않으며 전체 범위(-1..+1)로 출력합니다.

> **Note** 간단히 말해, 이 믹서의 출력은 SERVO = ( (좌우 회전각 입력 \* -0.6 + 0) \* 1 + (상하 회전각 입력 \* 0.65 + 0) \* 1 ) \* 1 + 0 가 됩니다.

보이는 모습 뒤에는 두 계수를 추가하는데, 비행익에 대해 좌우 회전각에 대해 최대 60% 감소, 상하 회전각에 대해 65% 감소함을 의미합니다.

완전한 믹서 동작은 다음과 같습니다:

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

에어프레임 "그룹"은 [QGroundControl](https://docs.qgroundcontrol.com/en/SetupView/Airframe.html)과 *에어프레임 참조*문서([PX4 개발 안내서](../airframes/airframe_reference.md) 와 [PX4 사용자 안내서](https://docs.px4.io/master/en/airframes/airframe_reference.html)) 에서의 선택에 대한 유사 에어프레임 분류에 활용합니다. 모든 그룹에는 이름이 있으며 에어프레임 분류에 대한 일반 공간 기하 정보, 모터 수, 모터 회전 방향을 보여주는 관련 svg 이미지가 있습니다.

*QGroundControl*에서 활용하는 에어프레임 메타데이터 파일과 문서 소스코드는 `make airframe_metadata` 명령으로 에어프레임 설명, 스크립트에서 만듭니다.

기존 그룹에 새 에어프레임을 넣을 때, [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d)에 있는 에어프레임 설명에 무언가를 더 추가할 필요는 없습니다.

에어프레임에 넣을 **새 그룹**이 추가로 필요하다면:

1. 분류에 해당하는 svg 이미지를 문서 저장소에 추가하십시오(이미지를 넣지 않으면 삽입 안내 이미지가 뜹니다): 
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