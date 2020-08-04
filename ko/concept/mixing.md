# 믹싱과 액추에이터

<!-- there is a useful doc here that we should still mine to further improve this topic: https://docs.google.com/document/d/1xCEQh48uDWyo7TjqedW6gYxBxMtNyuYZ2Xkt2MBb2-w -->

PX4 구조는 코어 컨트롤러에서 에어프레임 레이아웃이 특별한 케이스에 처리를 필요로 하지 않는 것을 보장합니다.

믹싱은 물리적 명령어 (예. `turn right`)를 받아들이고 그것을 모터 컨트롤이나 서보 컨트롤과 같은 액추에이터 명령어로 변환합니다. 에일러론당 하나의 서보를 가진 비행기의 경우 하나는 높게 다른 하나는 낮게 명령하는 것을 의미합니다. 멀티콥터에도 동일하게 적용됩니다. 앞으로 피칭하기 위해서는 모든 모터의 속도 변화가 필요합니다.

실제 자세 컨트롤러부터 믹서의 기능을 모듈화 시키는 것은 재사용성을 증가시킵니다.

## 파이프라인 컨트롤

특정 컨트롤러는 특정 정규화된 물리력이나 토크를 (-1..+1 로 스케일 됨) 믹서로 보내고, 그러면 각각의 액추에이터들이 설정됩니다. 출력 드라이버 (예. UART, UAVCAN 또는 PWM) 은 그것을 액추에이터의 기본 단위로 변환합니다 (예. 1300의 PWM 값).

![Mixer Control Pipeline](../../assets/concepts/mermaid_mixer_control_pipeline.png) <!--- Mermaid Live Version:
https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIGF0dF9jdHJsW0F0dGl0dWRlIENvbnRyb2xsZXJdIC0tPiBhY3RfZ3JvdXAwW0FjdHVhdG9yIENvbnRyb2wgR3JvdXAgMF1cbiAgZ2ltYmFsX2N0cmxbR2ltYmFsIENvbnRyb2xsZXJdIC0tPiBhY3RfZ3JvdXAyW0FjdHVhdG9yIENvbnRyb2wgR3JvdXAgMl1cbiAgYWN0X2dyb3VwMCAtLT4gb3V0cHV0X2dyb3VwNVtBY3R1YXRvciA1XVxuICBhY3RfZ3JvdXAwIC0tPiBvdXRwdXRfZ3JvdXA2W0FjdHVhdG9yIDZdXG4gIGFjdF9ncm91cDJbQWN0dWF0b3IgQ29udHJvbCBHcm91cCAyXSAtLT4gb3V0cHV0X2dyb3VwMFtBY3R1YXRvciA1XVxuXHRcdCIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In19
graph LR;
  att_ctrl[Attitude Controller] dash-dash> act_group0[Actuator Control Group 0]
  gimbal_ctrl[Gimbal Controller] dash-dash> act_group2[Actuator Control Group 2]
  act_group0 dash-dash> output_group5[Actuator 5]
  act_group0 dash-dash> output_group6[Actuator 6]
  act_group2[Actuator Control Group 2] dash-dash> output_group0[Actuator 5]
--->

## 제어 분류 

PX4는 제어 분류 (입력) 과 출력 분류를 활용합니다. 개념은 아주 간단합니다: 예를 들어 중요 비행 컨트롤러에 대한 제어 분류는 `attitude`, 페이로드 분류는 `gimbal` 입니다. 출력 분류는 하나의 물리적인 버스입니다 (예. 서보의 첫 8개의 PWM 출력). 이들 각 분류는 믹서에 대응하여 스케일할 수 있는 8개의 정규화 (-1..+1) 명령 포트가 있습니다. 하나의 믹서는 어떻게 8개의 제어 신호 각각을 8개의 출력으로 연결할지 정의합니다.

간단한 비행기를 예로 들면, 컨트롤 0 (rolle) 은 곧바로 출력 0 (aileron) 에 연결됩니다. 멀티콥터는 조금 다릅니다. 제어 0번(좌우 회전각)은 4개의 모터에 모두 연결하고 스로틀과 결합합니다.

### 제어 분류 #0 (비행 제어)

- 0: 좌우 회전각 (-1..1)
- 1: 상하 회전각 (-1..1)
- 2: 방위 회전각 (-1..1)
- 3: 스로틀 (0..1 일반 범위, -1..1 다양한 상하 회전각 조절 장치 / 역추진 장치용)
- 4: 플랩 (-1..1)
- 5: 스포일러 (-1..1)
- 6: 에어 제동장치 (-1..1)
- 7: 랜딩 기어 (-1..1)

### 제어 분류 #1 (수직이착륙기 비행제어/대체용)

- 0: 좌우 회전각 대체용 (-1..1)
- 1: 상하 회전각 대체용 (-1..1)
- 2: 방위 회전각 대체용 (-1..1)
- 3: 스로틀 대체용 (0..1 일반 범위, -1..1 다양한 상하 회전각 조절 장치 / 역추진 장치용 )
- 4: 예약 / aux0
- 5: 예약 / aux1
- 6: 예약 / aux2
- 7: 예약 / aux3

### 컨트롤 그룹 #2 (Gimbal)

- 0: 짐벌 좌우 회전각
- 1: 짐벌 상하 회전각
- 2: 짐벌 방위 회전각
- 3: 짐벌 셔터
- 4: 카메라 확대/축소
- 5: 예약
- 6: 예약
- 7: 예약 (패러슈트, -1..1)

### 제어 분류 #3 (수동 처리)

- 0: 원격 조종 좌우 회전각
- 1: 원격 조종 상하 회전각
- 2: 원격 조종 방위 회전각
- 3: 원격 조종 스로틀
- 4: 원격 조종 모드 전환 ([RC_MAP_FLAPS](../advanced/parameter_reference.md#RC_MAP_FLAPS)에 매핑한 RC 채널 통과)
- 5: 원격 조종 AUX 1 ([RC_MAP_AUX1](../advanced/parameter_reference.md#RC_MAP_AUX1)에 매핑한 RC 채널 통과)
- 6: 원격 조종 AUX2 ([RC_MAP_AUX2](../advanced/parameter_reference.md#RC_MAP_AUX2)에 매핑한 RC 채널 통과)
- 7: 원격 조종 AUX3 ([RC_MAP_AUX3](../advanced/parameter_reference.md#RC_MAP_AUX3)에 매핑한 RC 채널 통과)

> **Note** 이 그룹은 *일반 동작*을 진행하는 동안 특정 출력에 대한 원격 조정 대응 입력을 정의하는 용도로만 사용합니다(믹서에서 스케일링 처리하는 AUX2 예제는 [quad_x.main.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/quad_x.main.mix#L7)를 참고하십시오). 수동 입출력 이벤트 발생시 안전장치는 (PX4FMU 가 PX4IO 보드와의 통신을 멈췄을 때) 제어 그룹 0에 정의한 좌우/상하/방위 회전각 조절, 스로틀에 대한 매핑/믹싱만 활용합니다(다른 매핑은 무시).

### 제어 분류 #6 (첫번째 페이로드) {#control_group_6}

- 0: function 0
- 1: function 1
- 2: function 2
- 3: function 3
- 4: function 4
- 5: function 5
- 6: function 6
- 7: function 7

## 가상 제어 분류

> **Caution** *가상 제어 분류*는 수직이착륙기 코드를 작성하려는 개발자와 관련된 부분입니다. 믹서에서 사용하면 안되며, "완벽성"을 목적으로만 제공합니다.

이 그룹은 믹서의 입력을 받지 않습니다. 다만, 고정익과 멀티콥터 컨트롤러의 출력을 VTOL 거버너 모듈로 먹이려는 메타채널로 제공합니다.

### 제어 분류 #4 (비행 제어 MC 가상)

- 0: 좌우 회전각 대체용 (-1..1)
- 1: 상하 회전각 대체용 (-1..1)
- 2: 방위 회전각 대체용 (-1..1)
- 3: 스로틀 대체용 (0..1 일반 범위, -1..1 다양한 상하 회전각 조절 장치 / 역추진 장치용 )
- 4: 예약 / aux0
- 5: 예약 / aux1
- 6: 예약 / aux2
- 7: 예약 / aux3

### 제어 분류 #5 (비행 제어 FW 가상)

- 0: 좌우 회전각 대체용 (-1..1)
- 1: 상하 회전각 대체용 (-1..1)
- 2: 방위 회전각 대체용 (-1..1)
- 3: 스로틀 대체용 (0..1 일반 범위, -1..1 다양한 상하 회전각 조절 장치 / 역추진 장치용 )
- 4: 예약 / aux0
- 5: 예약 / aux1
- 6: 예약 / aux2
- 7: 예약 / aux3

## 출력 그룹/매핑

하나의 출력 분류는 믹서로 대응하고 스케일링할 수 있는 N개의(보통 8개) 정규화(-1..+1) 명령 포트를 가진 하나의 물리 버스(예: FMU PWM 출력, 입출력 PWM 출력, UAVCAN 등)입니다.

믹서 파일은 출력을 적용하는 실제 *출력 분류* (물리 버스)를 분명하게 정의하지 않습니다. 대신에, 믹서의 목적은 (예: MAIN 또는 AUX 출력 제어) 믹서 [파일 이름](#mixer_file_names)에서 알 수 있고, 시스템 [시작 스크립트](../concept/system_startup.md)에서 적절한 물리 버스로 대응합니다 ([rc.interface](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rc.interface) 에서 특정지음).

> **Note** This approach is needed because the physical bus used for MAIN outputs is not always the same; it depends on whether or not the flight controller has an IO Board (see [PX4 Reference Flight Controller Design > Main/IO Function Breakdown](../hardware/reference_design.md#mainio-function-breakdown)) or uses UAVCAN for motor control. The startup scripts load the mixer files into the appropriate device driver for the board, using the abstraction of a "device". The main mixer is loaded into device `/dev/uavcan/esc` (uavcan) if UAVCAN is enabled, and otherwise `/dev/pwm_output0` (this device is mapped to the IO driver on controllers with an I/O board, and the FMU driver on boards that don't). The aux mixer file is loaded into device `/dev/pwm_output1`, which maps to the FMU driver on Pixhawk controllers that have an I/O board.

여러개의 컨트롤 그룹과 (비행 컨트롤, 페이로드 등) 출력 그룹 (버스들) 이 있기 때문에, 하나의 컨트롤 그룹은 여러개의 출력 그룹에게 명령어를 보낼 수 있습니다.

![Mixer Input/Output Mapping](../../assets/concepts/mermaid_mixer_inputs_outputs.png) <!--- Mermaid Live Version:
https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGFjdHVhdG9yX2dyb3VwXzAtLT5vdXRwdXRfZ3JvdXBfNVxuICBhY3R1YXRvcl9ncm91cF8wLS0-b3V0cHV0X2dyb3VwXzZcbiAgYWN0dWF0b3JfZ3JvdXBfMS0tPm91dHB1dF9ncm91cF8wIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0
graph TD;
  actuator_group_0 dashdash>output_group_5
  actuator_group_0dashdash>output_group_6
  actuator_group_1dashdash>output_group_0
--->

> **Note** In practice, the startup scripts only load mixers into a single device (output group). This is a configuration rather than technical limitation; you could load the main mixer into multiple drivers and have, for example, the same signal on both UAVCAN and the main pins.

## PX4 믹서 정의

Mixers are defined in plain-text files using the [syntax](#mixer_syntax) below.

Files for pre-defined airframes can be found in [ROMFS/px4fmu_common/mixers](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/mixers). These can be used as a basis for customisation, or for general testing purposes.

### 믹서 파일 이름 {#mixer_file_names}

A mixer file must be named **XXXX.*main*.mix** if it is responsible for the mixing of MAIN outputs or **XXXX.*aux*.mix** if it mixes AUX outputs.

### 믹서 불러오기 {#loading_mixer}

The default set of mixer files (in Firmware) are defined in [px4fmu_common/init.d/airframes/](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/airframes/). These can be overridden by mixer files with the same name in the SD card directory **/etc/mixers/** (SD card mixer files are loaded by preference).

PX4 loads mixer files named **XXXX.*main*.mix** onto the MAIN outputs and **YYYY.*aux*.mix** onto the AUX outputs, where the prefixes depend on the airframe and airframe configuration. Commonly the MAIN and AUX outputs correspond to MAIN and AUX PWM outputs, but these may be loaded into a UAVCAN (or other) bus when that is enabled.

The MAIN mixer filename (prefix `XXXX`) is set in the airframe configuration using `set MIXER XXXX` (e.g. [airframes/10015_tbs_discovery](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/airframes/10015_tbs_discovery) calls `set MIXER quad_w` to load the main mixer file **quad_w.*main*.mix**).

The AUX mixer filename (prefix `YYYY` above) depends on airframe settings and/or defaults:

- `MIXER_AUX` can be used to *explicitly* set which AUX file is loaded (e.g. in the aiframe configuration, `set MIXER_AUX vtol_AAERT` will load `vtol_AAERT.aux.mix`).
- Multicopter and Fixed-Wing airframes load [pass.aux.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/pass.aux.mix) by default (i.e if not set using `MIXER_AUX`). > **Tip** `pass.aux.mix` is the *RC passthrough mixer*, which passes the values of 4 user-defined RC channels (set using the [RC_MAP_AUXx/RC_MAP_FLAPS](../advanced/parameter_reference.md#RC_MAP_AUX1) parameters) to the first four outputs on the AUX output.
- VTOL frames load the AUX file specified using `MIXER_AUX` if set, or the value specified by `MIXER` if not.
- Frames with gimbal control enabled (and output mode set to AUX) will *override* the airframe-specific MIXER_AUX setting and load `mount.aux.mix` on the AUX outputs.

> **Note** Mixer file loading is implemented in [ROMFS/px4fmu_common/init.d/rc.interface](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rc.interface).

### 개별 믹서 불러오기 {#loading_custom_mixer}

PX4 loads appropriately named mixer files from the SD card directory **/etc/mixers/**, by preference, and then the version in Firmware.

To load a custom mixer, you should give it the same name as a "normal" mixer file (that is going to be loaded by your airframe) and put it in the **etc/mixers** directory on your flight controller's SD card.

Most commonly you will override/replace the **AUX** mixer file for your current airframe (which may be the RC passthrough mixer - [pass.aux.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/pass.aux.mix)). See above for more information on [mixer loading](#loading_mixer).

> **Tip** You can also *manually* load a mixer at runtime using the [mixer load](../middleware/modules_command.md#mixer) command (thereby avoiding the need for a reboot). For example, to load a mixer **/etc/mixers/test_mixer.mix** onto the MAIN PWM outputs, you could enter the following command in a [console](../debug/consoles.md): ```mixer load /dev/pwm_output0 /fs/microsd/etc/mixers/test_mixer.mix```

### 문법 {#mixer_syntax}

믹서 파일은 하나 이상의 믹서를 정의하는 텍스트 파일입니다: 하나 이상의 입력과 출력을 서로 대응합니다.

믹서 정의 형식에는 [멀티로터 믹서](#multirotor_mixer), [헬리콥터 믹서](#helicopter_mixer), [결합 믹서](#summing_mixer), [널 믹서](#null_mixer) 네가지가 있습니다.

- [멀티로터 믹서](#multirotor_mixer) - + 방향 또는 X축 방향의 4, 6, 8 회전 객체 출력을 가진 기체를 정의
- [헬리콥터 믹서](#helicopter_mixer) - 경사판 서보와 메인 모터 ESC 출력을 지닌 기체를 정의(미익은 별도의 [결합 믹서](#summing_mixer)임.)
- [결합 믹서](#summing_mixer) - 0개 이상의 제어 입력을 단일 액츄에이터 출력으로 결합합니다. 입력은 비례 조정하며, 출력 계수를 반영하기 전에 믹싱 함수에서 결합합니다.
- [널 믹서](#null_mixer) - 0을 출력하는 단일 액츄에이터 출력을 만듭니다(안전장치 모드가 아닐 때).

> **Tip** Use *multirotor* and *helicopter mixers* for the respective types, the *summing mixer* for servos and actuator controls, and the *null mixer* for creating outputs that must be zero during normal use (e.g. a parachute has 0 normally, but might have a particular value during failsafe). A [VTOL Mixer](#vtol_mixer) combines the other mixer types.

각 믹서에서 내보내는 여러 출력은 믹서 형식과 설정에 따라 다릅니다. 예를 들어 결합 믹서나 널 믹서의 경우는 하나의 출력만 내보내는데 반해, 멀티로터 믹서는 공간 기하 정보에 따라 4, 6, 8개의 출력을 내보냅니다.

각 파일에는 하나 이상의 믹서를 지정할 수 있습니다. 출력 순서(액츄에이터로의 믹서 할당)는 믹서 정의를 읽는 장비에 따라 다릅니다. PWM 장비의 경우, 출력 순서는 선언 순서에 일치합니다. 예를 들어 성분 네개의 공간 기하 정보를 지닌 멀티로터 믹서를 지정하고, 그 다음 널 믹서를, 그 다음은 두개의 결합 믹서를 지정하면, 처음 네개의 출력을 네개의 액츄에이터로, 그 다음은 "빈" 출력을, 그 다음은 두개의 출력이 뒤따라옵니다.

각각의 믹서 정의는 다음 형식의 행으로 시작합니다:

    <tag>: <mixer arguments>
    

`tag`로 믹서 형식을 선택합니다(각 형식에 대한 자세한 내용은 링크 참고):

- `R`: [멀티로터 믹서](#multirotor_mixer)
- `H`: [헬리콥터 믹서](#helicopter_mixer)
- `M`: [결합 믹서](#summing_mixer)
- `Z`: [널 믹서](#null_mixer)

일부 믹서는 믹서 형식 태그가 뒤따라오는 식으로 여러 태그를 구성(예: `O`, `S`)하여 정의하기도 합니다.

> **Note** 단일 대문자로 시작하지 않은 일부 행은 무시합니다(따라서 설명문을 정의 행과 자유롭게 섞어쓸 수 있습니다).

#### 믹서 결합 {#summing_mixer}

결합 믹서는 액츄에이터와 서보 제어 목적으로 활용합니다.

(단순) 결합 믹서는 0개 이상의 제어 입력을 단일 액츄에이터 출력으로 결합합니다. 입력값은 스케일링 처리하고, 믹싱 함수는 출력 계수에 적용하기 전에 결과를 결합합니다.

단순 믹서 정의는 다음처럼 시작합니다:

    M: <control count>
    O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

`<control count>` 값이 0이면, 결합 결과값은 0값이며, 믹서에서는 `<lower limit>` 값과 `<upper limit>` 값으로 제한한 `<offset>` 값이 나옵니다.

둘째 행에서는 위에서 언급한대로 계수 매개변수로 출력 계수를 지정합니다. 소숫점 처리 계산을 진행하는 동안 정의 파일의 값은 상수 10000만배만큼 늘어납니다. 예를 들어 -0.5 오프셋은 -5000으로 인코딩합니다.

정의 내용을 통해 다음과 같은 형식으로 `<control count>`개 항목에 대한 제어 입력, 스케일링을 지정합니다:

    S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

> **Note** `S:` 행은 `O:` 행 아래에 있어야합니다.

`<group>` 값은 계수를 읽을 제어 분류 식별자를 정의하며, `<index>` 값은 해당 그룹의 오프셋을 지정합니다. 이 값은 믹서 정의를 장치에 불러올 때 장치에 해당하는 값입니다.

When used to mix vehicle controls, mixer group zero is the vehicle attitude control group, and index values zero through three are normally roll, pitch, yaw and thrust respectively.

The remaining fields on the line configure the control scaler with parameters as discussed above. Whilst the calculations are performed as floating-point operations, the values stored in the definition file are scaled by a factor of 10000; i.e. an offset of -0.5 is encoded as -5000.

An example of a typical mixer file is explained [here](../airframes/adding_a_new_frame.md#mixer-file).

#### 널(null) 믹서 {#null_mixer}

널 믹서는 어떤 제어도 받지 않으며 항상 0값만 출력하는 단일 액츄에이터를 생성합니다.

보통 널 믹서는 특정 액츄에이터 출력 패턴을 만들기 위해 믹서 집합에서 빈 자리로 활용합니다 안전 장치에 사용하는 출력 값을 제어하는 용도로도 사용할 수 있습니다(보통 출력 값은 0 입니다. 안전장치 가동시 믹서는 무시하며 안전장치 처리용 값을 대신 활용합니다).

널 믹서 정의는 다음과 같습니다:

    Z:
    

#### 멀티로터 믹서 {#multirotor_mixer}

멀티로터 믹서는 컨트롤러 입력 넷(좌우/상하/방위 회전각, 역추진)을 모터 속도 컨트롤러를 제어할 액츄에이터 출력조합으로 모읍니다.

믹서 정의는 단일 행의 형태를 지니고 있습니다:

    R: <geometry> <roll scale> <pitch scale> <yaw scale> <idlespeed>
    

지원하는 공간 기하 성분은 다음과 같습니다:

- 4x - quadrotor in X configuration
- 4+ - quadrotor in + configuration
- 6x - hexacopter in X configuration
- 6+ - hexacopter in + configuration
- 8x - octocopter in X configuration
- 8+ - octocopter in + configuration

Each of the roll, pitch and yaw scale values determine scaling of the roll, pitch and yaw controls relative to the thrust control. Whilst the calculations are performed as floating-point operations, the values stored in the definition file are scaled by a factor of 10000; i.e. an factor of 0.5 is encoded as 5000.

Roll, pitch and yaw inputs are expected to range from -1.0 to 1.0, whilst the thrust input ranges from 0.0 to 1.0. Output for each actuator is in the range -1.0 to 1.0.

Idlespeed can range from 0.0 to 1.0. Idlespeed is relative to the maximum speed of motors and it is the speed at which the motors are commanded to rotate when all control inputs are zero.

액츄에이터 출력 값이 임계에 도달할 경우, 모든 액츄에이터 출력 값을 다시 비례 조정하여 임계에 도달한 액츄에이터 출력 값은 1.0으로 제한됩니다.

#### 헬리콥터 믹서  {#helicopter_mixer}

The helicopter mixer combines three control inputs (roll, pitch, thrust) into four outputs (swash-plate servos and main motor ESC setting). The first output of the helicopter mixer is the throttle setting for the main motor. The subsequent outputs are the swash-plate servos. The tail-rotor can be controlled by adding a simple mixer.

The thrust control input is used for both the main motor setting as well as the collective pitch for the swash-plate. It uses a throttle-curve and a pitch-curve, both consisting of five points.

> **Note** The throttle- and pitch- curves map the "thrust" stick input position to a throttle value and a pitch value (separately). This allows the flight characteristics to be tuned for different types of flying. An explanation of how curves might be tuned can be found in [this guide](https://www.rchelicopterfun.com/rc-helicopter-radios.html) (search on *Programmable Throttle Curves* and *Programmable Pitch Curves*).

The mixer definition begins with:

    H: <number of swash-plate servos, either 3 or 4>
    T: <throttle setting at thrust: 0%> <25%> <50%> <75%> <100%>
    P: <collective pitch at thrust: 0%> <25%> <50%> <75%> <100%>
    

`T:` defines the points for the throttle-curve. `P:` defines the points for the pitch-curve. Both curves contain five points in the range between 0 and 10000. For simple linear behavior, the five values for a curve should be `0 2500 5000 7500 10000`.

This is followed by lines for each of the swash-plate servos (either 3 or 4) in the following form:

    S: <angle> <arm length> <scale> <offset> <lower limit> <upper limit>
    

The `<angle>` is in degrees, with 0 degrees being in the direction of the nose. Viewed from above, a positive angle is clock-wise. The `<arm length>` is a normalized length with 10000 being equal to 1. If all servo-arms are the same length, the values should al be 10000. A bigger arm length reduces the amount of servo deflection and a shorter arm will increase the servo deflection.

The servo output is scaled by `<scale> / 10000`. After the scaling, the `<offset>` is applied, which should be between -10000 and +10000. The `<lower limit>` and `<upper limit>` should be -10000 and +10000 for full servo range.

The tail rotor can be controller by adding a [summing mixer](#summing_mixer):

    M: 1
    S: 0 2  10000  10000      0 -10000  10000
    

By doing so, the tail rotor setting is directly mapped to the yaw command. This works for both servo-controlled tail-rotors, as well as for tail rotors with a dedicated motor.

The [blade 130 helicopter mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/blade130.main.mix) can be viewed as an example.

    H: 3
    T:      0   3000   6000   8000  10000
    P:    500   1500   2500   3500   4500
    # Swash plate servos:
    S:      0  10000  10000      0  -8000   8000
    S:    140  13054  10000      0  -8000   8000
    S:    220  13054  10000      0  -8000   8000
    
    # Tail servo:
    M: 1
    S: 0 2  10000  10000      0 -10000  10000
    

- The throttle-curve starts with a slightly steeper slope to reach 6000 (0.6) at 50% thrust.
- It continues with a less steep slope to reach 10000 (1.0) at 100% thrust.
- The pitch-curve is linear, but does not use the entire range.
- At 0% throttle, the collective pitch setting is already at 500 (0.05).
- At maximum throttle, the collective pitch is only 4500 (0.45).
- Using higher values for this type of helicopter would stall the blades.
- The swash-plate servos for this helicopter are located at angles of 0, 140 and 220 degrees.
- The servo arm-lenghts are not equal.
- The second and third servo have a longer arm, by a ratio of 1.3054 compared to the first servo.
- The servos are limited at -8000 and 8000 because they are mechanically constrained.

#### VTOL 믹서 {#vtol_mixer}

VTOL 시스템은 멀티로터 출력 목적의 [멀티로터 믹서](#multirotor_mixer)를 활용하며, 고정익 액츄에이터 동작을 위해 [결합 믹서](#summing_mixer)를 채용했습니다(그리고 틸트로터 수직 이착륙기의 경우 틸팅 서보를 결합 믹서에 붙입니다).

수직 이착륙기 믹서 시스템은 단일 믹서로 결합할 수 있는데, 액츄에이터는 입출력 포트 또는 FMU 포트로 연결하거나, 제각각의 믹서 파일로 입출력과 AUX용포트를 따로 나눕니다. 액츄에이터를 별도로 분리할 경우, 모든 멀티콥터 모터를 하나의 포트에 모아 붙이고, 모든 서보와 고정익 모터를 다른 포트에 모아 붙이는 방안을 추천드립니다.

> **Note** The FMU output can only be used for multirotor motors starting from PX4 v1.11. To use the FMU output set [VT_MC_ON_FMU=1](../advanced/parameter_reference.md#VT_MC_ON_FMU) (otherwise they are not switched off when in fixed-wing flight mode).