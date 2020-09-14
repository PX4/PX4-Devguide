# 믹싱과 액추에이터

<!-- there is a useful doc here that we should still mine to further improve this topic: https://docs.google.com/document/d/1xCEQh48uDWyo7TjqedW6gYxBxMtNyuYZ2Xkt2MBb2-w -->

PX4 구조는 코어 컨트롤러에서 에어프레임 레이아웃이 특별한 케이스에 처리를 필요로 하지 않는 것을 보장합니다.

믹싱은 물리적 명령어 (예. `turn right`)를 받아들이고 그것을 모터 컨트롤이나 서보 컨트롤과 같은 액추에이터 명령어로 변환합니다. 에일러론당 하나의 서보를 가진 비행기의 경우 하나는 높게 다른 하나는 낮게 명령하는 것을 의미합니다. 멀티콥터에도 동일하게 적용됩니다. 앞으로 피칭하기 위해서는 모든 모터의 속도 변화가 필요합니다.

실제 자세 컨트롤러부터 믹서의 기능을 모듈화 시키는 것은 재사용성을 증가시킵니다.

## 파이프라인 컨트롤

특정 컨트롤러는 특정 정규화된 물리력이나 토크를 (-1..+1 로 스케일 됨) 믹서로 보내고, 그러면 각각의 액추에이터들이 설정됩니다. 출력 드라이버 (예. UART, UAVCAN 또는 PWM) 은 그것을 액추에이터의 기본 단위로 변환합니다 (예. 1300의 PWM 값).

![믹서 제어 파이프라인](../../assets/concepts/mermaid_mixer_control_pipeline.png) <!--- Mermaid Live Version:
https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIGF0dF9jdHJsW0F0dGl0dWRlIENvbnRyb2xsZXJdIC0tPiBhY3RfZ3JvdXAwW0FjdHVhdG9yIENvbnRyb2wgR3JvdXAgMF1cbiAgZ2ltYmFsX2N0cmxbR2ltYmFsIENvbnRyb2xsZXJdIC0tPiBhY3RfZ3JvdXAyW0FjdHVhdG9yIENvbnRyb2wgR3JvdXAgMl1cbiAgYWN0X2dyb3VwMCAtLT4gb3V0cHV0X2dyb3VwNVtBY3R1YXRvciA1XVxuICBhY3RfZ3JvdXAwIC0tPiBvdXRwdXRfZ3JvdXA2W0FjdHVhdG9yIDZdXG4gIGFjdF9ncm91cDJbQWN0dWF0b3IgQ29udHJvbCBHcm91cCAyXSAtLT4gb3V0cHV0X2dyb3VwMFtBY3R1YXRvciA1XVxuXHRcdCIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In19
graph LR;
  att_ctrl[Attitude Controller] dash-dash> act_group0[Actuator Control Group 0]
  gimbal_ctrl[Gimbal Controller] dash-dash> act_group2[Actuator Control Group 2]
  act_group0 dash-dash> output_group5[Actuator 5]
  act_group0 dash-dash> output_group6[Actuator 6]
  act_group2[Actuator Control Group 2] dash-dash> output_group0[Actuator 5]
--->

## 제어 분류 

PX4는 제어 분류 (입력) 과 출력 분류를 활용합니다. 개념은 아주 간단합니다: 예를 들어 핵심 비행체 제어 장치에 대한 제어 분류는 `attitude`, 탑재 분류는 `gimbal` 입니다. 출력 분류는 하나의 물리적인 버스입니다 (예. 서보의 첫 8개의 PWM 출력). 이들 각 분류는 믹서에 대응하여 스케일할 수 있는 8개의 정규화 (-1..+1) 명령 포트가 있습니다. 하나의 믹서는 어떻게 8개의 제어 신호 각각을 8개의 출력으로 연결할지 정의합니다.

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

### 제어 분류 #1 (수직 이착륙 비행 제어/대체용)

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

### 제어 분류 #6 (첫번째 탑재 장치) {#control_group_6}

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

이 그룹은 믹서의 입력을 받지 않습니다. 다만, 고정익과 멀티콥터 컨트롤러의 출력을 수직 이착륙 거버너 모듈로 먹이려는 메타채널로 제공합니다.

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

> **Note** MAIN 출력에 활용하는 물리 버스가 항상 동일하지 않으므로 이런 접근 방법이 필요합니다. 비행체 제어 장치에 입출력 보드가 붙어([PX4 레퍼런스 비행체 제어 장치 설계 > 메인 입출력 기능 해부](../hardware/reference_design.md#mainio-function-breakdown)편을 참고)있거나 모터 제어 목적으로 UAVCAN 통신 수단을 활용하는지 여부에 따라 달려있습니다. 시작 스크립트에서는 "device" 추상 레이어를 활용하여 적절한 믹서 파일을 보드에 적당한 장치 드라이버로 불러옵니다. 메인 믹서는 UAVCAN을 활성화했을 경우 `/dev/uavcan/esc`(uavcan) 장치를 불러오며, 그렇지 않을 경우 `/dev/pwm_output0`(이 장치는 입출력 보드 조종기의 입출력 드라이버를 대응하며, 보드의 FMU 드라이버는 이에 해당하지 않습니다) 조종 장치를 불러옵니다. AUX 믹서 파일은 입출력 보드를 내장한 픽스호크 컨트롤러의 FMU 드라이버에 대응하는 `/dev/pwm_output1` 장치에 불러옵니다.

여러 제어 분류와 (비행체 제어, 탑재 장치 등) 출력 분류(여러 버스)가 있기 때문에, 하나의 제어 분류를 여러 개의 출력에 명령을 보낼 수 있습니다.

![믹서 입출력 대응](../../assets/concepts/mermaid_mixer_inputs_outputs.png) <!--- Mermaid Live Version:
https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGFjdHVhdG9yX2dyb3VwXzAtLT5vdXRwdXRfZ3JvdXBfNVxuICBhY3R1YXRvcl9ncm91cF8wLS0-b3V0cHV0X2dyb3VwXzZcbiAgYWN0dWF0b3JfZ3JvdXBfMS0tPm91dHB1dF9ncm91cF8wIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0
graph TD;
  actuator_group_0 dashdash>output_group_5
  actuator_group_0dashdash>output_group_6
  actuator_group_1dashdash>output_group_0
--->

> **Note** 실제로 시작 스크립트는 단일 장치(출력 분류)에 믹서만 불러옵니다. 기술적인 제한이라기보단 그냥 설정입니다. 여러 드라이버에 메인 믹서를 불러올 수 있습니다. 예를 들면 메인 믹서를 통해 UAVCAN과 메인 핀에 동일한 신호를 줍니다.

## PX4 믹서 정의

믹서는 아래 [문법](#mixer_syntax)을 따라 플레인 텍스트로 정의합니다.

사전 정의한 에어프레임 파일은 [ROMFS/px4fmu_common/mixers](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/mixers)에 있습니다. 이 구성요소는 개별 설정 기반 또는 일반 시험 목적으로 활용할 수 있습니다.

### 믹서 파일 이름 {#mixer_file_names}

믹서 파일은 MAIN 출력 혼합에 해당할 경우**XXXX.*main*.mix**, AUX 출력 혼합에 해당할 경우 **XXXX.*aux*.mix**로 이름을 붙여야 합니다.

### 믹서 불러오기 {#loading_mixer}

(펌웨어의) 믹서 파일 기본 모음은 [px4fmu_common/init.d/airframes/](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/airframes/)에 지정합니다. SD카드 메모리에 있는 **/etc/mixers** 디렉터리에 동일한 이름을 가진 믹서 파일을 두어 우선 중복적용할 수 있습니다(SD 카드 믹서 파일은 기본설정으로 불러옴).

PX4에서는 MAIN 출력에 해당하는 파일의 이름을 **XXXX.*main*.mix**로, AUX 출력에 해당하는 파일 이름을 **YYYY.*aux*.mix**로 정하며, 여기서 접두부는 에어프레임과 에어프레임 설정에 따릅니다. 보통 MAIN 출력과 AUX 출력은 MAIN PWM 출력과 AUX PWM 출력에 해당하지만, UAVCAN(또는 기타) 버스를 활성화 했을 때는 UAVCAN으로 불러옵니다.

MAIN 믹서 파일 이름(앞에 `XXXX`가 붙음)은 `set MIXER XXXX` 설정행으로 에어프레임 설정에서 맞춥니다(예: [airframes/10015_tbs_discovery](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/airframes/10015_tbs_discovery)은 `set MIXER quad_w`를 호출하여 **quad_w.*main*.mix** 메인 믹서 파일을 불러옵니다).

AUX 믹서 파일 이름(위에서 `YYYY`로 앞에 붙음)은 에어프레임 설정이나 기본값 여부에 따릅니다:

- `MIXER_AUX`는 *분명하게* 어떤 AUX 파일을 불러올 지 설정할 때 활용할 수 있습니다(예: 에어프레임 설정시 `set MIXER_AUX vtol_AAERT` 설정은 `vtol_AAERT.aux.mix` 파일을 불러옴).
- 멀티콥터와 고정익 에어프레임은 기본적으로 [pass.aux.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/pass.aux.mix) 믹서 파일을 불러옵니다(예: 따로 설정하지 않으면 `MIXER_AUX` 활용). > **Tip** `pass.aux.mix` 파일은 *원격 조종 처리 믹서*이며, 믹서에서는 4개의 사용자 지정 원격 조종 채널 값을 ([RC_MAP_AUXx/RC_MAP_FLAPS](../advanced/parameter_reference.md#RC_MAP_AUX1) 매개변수 활용) AUX 출력의 첫번째 출력 넷으로 전달합니다.
- 수직 이착륙 프레임에 `MIXER_AUX`을 설정했을 경우 지정 AUX 파일을 불러오며, 그렇지 않을 경우 `MIXER`에 지정한 값대로 파일을 불러옵니다.
- 짐벌 조종간을 활용할 수 있(고 AUX에 출력 상태를 설정)는 프레임은 에어프레임별 MIXER_AUX 설정보다 *우선 반영*하며, `mount.aux.mix` 파일을 AUX 출력에 불러옵니다.

> **Note** 믹서 파일을 불러오는 부분은 [ROMFS/px4fmu_common/init.d/rc.interface](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rc.interface)에 있습니다.

### 개별 믹서 불러오기 {#loading_custom_mixer}

PX4는 SD 카드의 **/etc/mixers/** 디렉터리에서 적절한 이름이 붙은 믹서 파일을 기본 설정에 따라 불러온 후, 펌웨어 버전을 불러옵니다.

개별 정의 믹서를 불러오려면 "일반" 믹서 파일과 동일한 이름을 부여해야 하며(이 파일을 에어프레임에서 불러옵니다), 이 파일을 비행체 제어 장치의 SD 카드의 **/etc/mixers**에 넣어야합니다.

대부분 **AUX** 믹서 파일을 현재 에어프레임에 따라 대체합니다(원격 조종 처리 믹서 파일 이름은 [pass.aux.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/pass.aux.mix)가 됩니다). [믹서 불러오기](#loading_mixer)에 대한 자세한 정보는 위를 다시 살펴보십시오.

> **Tip** 실행 시간에 [mixer load](../middleware/modules_command.md#mixer) 명령으로 믹서를 *직접* 불러올 수도 있습니다(다시 부팅하는 상황을 피함). 예를 들면, MAIN PWM 출력의 **/etc/mixers/test_mixer.mix** 믹서 파일을 불러오려면, [콘솔](../debug/consoles.md)에서 다음 명령을 입력합니다: ```mixer load /dev/pwm_output0 /fs/microsd/etc/mixers/test_mixer.mix```

### 문법 {#mixer_syntax}

믹서 파일은 하나 이상의 믹서를 정의하는 텍스트 파일입니다: 하나 이상의 입력과 출력을 서로 대응합니다.

믹서 정의 형식에는 [멀티로터 믹서](#multirotor_mixer), [헬리콥터 믹서](#helicopter_mixer), [결합 믹서](#summing_mixer), [널 믹서](#null_mixer) 네가지가 있습니다.

- [멀티로터 믹서](#multirotor_mixer) - + 방향 또는 X축 방향의 4, 6, 8 회전 객체 출력을 가진 기체를 정의
- [헬리콥터 믹서](#helicopter_mixer) - 경사판 서보와 메인 모터 ESC 출력을 지닌 기체를 정의(후미익은 별도의 [결합 믹서](#summing_mixer)임.)
- [결합 믹서](#summing_mixer) - 0개 이상의 제어 입력을 단일 액츄에이터 출력으로 결합합니다. 입력은 비례 조정하며, 출력 계수를 반영하기 전에 믹싱 함수에서 결합합니다.
- [널 믹서](#null_mixer) - 0을 출력하는 단일 액츄에이터 출력을 만듭니다(안전장치 모드가 아닐 때).

> **Tip** *멀티로터*와 *헬리콥터 믹서*는 각 형식에 맞게 사용하십시오. *결합 믹서*는 서보와 액츄에이터 제어에 해당하며, *널 믹서*는 일반 사용시 0값을 출력해야 하는 경우의 출력을 만들 때 활용합니다(예: 낙하산은 보통 0 값을 주지만, 안전장치 가동시 해당 값을 부여합니다). [수직 이착륙 믹서](#vtol_mixer)는 다른 믹서 형식을 혼합합니다.

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

둘째 행에서는 위에서 언급한대로 계수 매개변수로 출력 계수를 지정합니다. 부동 소숫점 연산을 수행하는 동안 정의 파일에 지정한 값은 10000만큼 배율을 조정합니다. 예를 들어 -0.5 오프셋은 -5000으로 인코딩합니다.

정의 내용을 통해 다음과 같은 형식으로 `<control count>`개 항목에 대한 제어 입력, 스케일링을 지정합니다:

    S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

> **Note** `S:` 행은 `O:` 행 아래에 있어야합니다.

`<group>` 값은 계수를 읽을 제어 분류 식별자를 정의하며, `<index>` 값은 해당 그룹의 오프셋을 지정합니다. 이 값은 믹서 정의를 장치에 불러올 때 장치에 해당하는 값입니다.

기체 제어 입력을 믹싱할 때, 믹서 분류 0은 기체 고도 제어 분류이며, 0부터 3까지의 일반 인덱스 값은 각각, 좌우 회전각(roll), 상하 회전각(pitch), 방위 회전각(yaw), 추력입니다.

행의 나머지 필드에서는 위에서 다룬대로 매개변수를 통해 제어 계수를 설정합니다. 부동 소숫점 연산을 수행하는 동안 정의 파일에 지정한 값은 1만배 증가합니다. 예를 들어 -0.5 오프셋은 -5000으로 인코딩합니다.

일반적인 믹서 파일 예제는 [여기](../airframes/adding_a_new_frame.md#mixer-file)에서 설명합니다.

#### 널(null) 믹서 {#null_mixer}

널 믹서는 어떤 제어도 받지 않으며 항상 0값만 출력하는 단일 액츄에이터를 생성합니다.

보통 널 믹서는 특정 액츄에이터 출력 패턴을 만들기 위해 믹서 집합에서 빈 자리로 활용합니다 안전 장치에 사용하는 출력 값을 제어하는 용도로도 사용할 수 있습니다(보통 출력 값은 0 입니다. 안전장치 가동시 믹서는 무시하며 안전장치 처리용 값을 대신 활용합니다).

널 믹서 정의는 다음과 같습니다:

    Z:
    

#### 멀티로터 믹서 {#multirotor_mixer}

멀티로터 믹서는 컨트롤러 입력 넷(좌우/상하/방위 회전각, 추력)을 모터 속도 컨트롤러를 제어할 액츄에이터 출력조합으로 모읍니다.

믹서 정의는 단일 행의 형태를 지니고 있습니다:

    R: <geometry> <roll scale> <pitch scale> <yaw scale> <idlespeed>
    

지원하는 공간 기하 성분은 다음과 같습니다:

- 4x - 쿼드로터의 X 설정
- 4+ - 쿼드로터의 + 설정
- 6x - 헥사콥터의 X 설정
- 6+ - 헥사콥터의 + 설정
- 8x - 옥토콥터의 X 설정
- 8+ - 옥토콥터의 + 설정

각 좌우 회전각(roll), 상하 회전각(pitch), 방위 회전각(yaw) 계수 값은 추력 제어에 상대적인 좌우 회전각, 상하 회전각, 방위 회전각 제어의 배율을 결정합니다. 부동 소숫점 연산을 수행하는 동안 정의 파일에 지정한 값은 10000만큼 배율을 조정합니다. 예를 들어 -0.5 오프셋은 -5000으로 인코딩합니다. 

좌우 회전각(roll), 상하 회전각(pitch), 방위 회전각(yaw) 입력은 -1.0에서 1.0 정도 되나, 추력 입력 범위는 0.0에서 1.0 정도 됩니다. 각 액츄에이터 출력 범위는 -1.0에서 1.0까지입니다.

대기 속도는 0.0에서 1.0의 범위를 가질 수 있습니다. 대기 속도는 모터의 최대 속도에 대해 상대적이며, 다른 제어 입력을 0으로 하는 동안 어떤 모터를 회전하도록 명령을 내리느냐에 따른 속력이기도 합니다.

액츄에이터 출력 값이 임계에 도달할 경우, 모든 액츄에이터 출력 값을 다시 비례 조정하여 임계에 도달한 액츄에이터 출력 값은 1.0으로 제한됩니다.

#### 헬리콥터 믹서  {#helicopter_mixer}

헬리콥터 믹서는 세개의 제어 입력(좌우 회전각 - roll, 상하 회전각 - pitch, 추력 - thrust)을 네 개의 출력으로 묶습니다(경사판 모터와 메인 모터 ESC 설정). 헬리콥터 믹서의 첫 출력은 메인 모터의 스로틀 설정입니다. 그 다음 따라오는 출력은 경사판 서보용입니다. 후미익은 단일 믹서를 추가하여 제어할 수 있습니다.

추력 제어 입력은 경사판 모터의 상하 회전각 보정과 메인 모터 설정에 활용합니다. 스로틀 곡선과 상하 회전각 곡선을 활용하며, 커브 곡선은 5개의 점을 따라 구성합니다.

> **Note** 스로틀 출력 변화 곡선과 상하 회전각 변화 곡선은 포지션에 대응하여 스로틀 값과 상하 회전각 값(제각각 별도로)이 위치한 "추력" 스틱 입력에 대응합니다. 이 설정 방식으로 제각기 다른 비행체 형식을 지닌 비행체의 각 부분의 특징을 조정할 수 있습니다. 커브 조정 방법 설명은 [이 안내서](https://www.rchelicopterfun.com/rc-helicopter-radios.html)에 있습니다 (*프로그램 구성 가능한 스로틀 출력 변화 곡선*과 *프로그램 구성 가능한 상하 회전각 변화 곡선*에서 찾아보십시오).

믹서 정의는 다음으로 시작합니다:

    H: <number of swash-plate servos, either 3 or 4>
    T: <throttle setting at thrust: 0%> <25%> <50%> <75%> <100%>
    P: <collective pitch at thrust: 0%> <25%> <50%> <75%> <100%>
    

`T:` 스로틀 출력 변화 곡선 점을 정의합니다. `P:` 상하 회전각 변화 곡선 점을 정의합니다. 두 변화 곡선 설정은 0부터 10000까지의 점 다섯 개를 보유합니다. 단순 선형 동작의 경우, 변화 곡선 다섯 점의 값은 `0 2500 5000 7500 10000`입니다.

아래는 각각의 (3개 또는 4개) 경사판 서보에 대한 행으로, 다음과 같은 형식을 갖추고 있습니다:

    S: <angle> <arm length> <scale> <offset> <lower limit> <upper limit>
    

`<angle>`는 도 단위로, 0도는 선두 부분의 방향입니다. 위에서 본 바와 같이, 양의 각도 방향은 시계 방향입니다. `<arm length>`은 정규화 길이이며, 10000은 1을 의미합니다. 모든 서보 암이 동일한 길이를 가진다면, 해당 값은 10000이어야 합니다. 암의 길이가 늘어나면 서보 하자율을 줄이며, 길이가 줄어들면 하자율이 늘어납니다.

서보 출력은 `<scale> / 10000`만큼 비율 조정됩니다. 비율 조정이 끝나면 `<offset>`을 반영하는데 이 값은 -10000에서 +10000까지입니다. `<lower limit>`과 `<upper limit>` 값은 전체 서보 출력 범위에 해당하는 -10000과 +10000입니다.

후미익은 [결합 믹서](#summing_mixer)를 추가하여 제어할 수 있습니다.

    M: 1
    S: 0 2  10000  10000      0 -10000  10000
    

이렇게 하여, 후미익 설정은 yaw 명령에 직접적으로 대응합니다. 후미익에 전용 모터가 달린만큼, 후미익 서보 제어 로터와 동작합니다.

[blade 130 helicopter mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/blade130.main.mix)를 예로 살펴볼 수 있습니다.

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
    

- 스로틀-커브는 50%의 추력에 6000 (0.6) 값에 도달하는 약간 가파른 경사로 시작합니다.
- 덜 가파른 경사로 계속 진행하며 100% 추력으로 10000 (1.0) 에 도달합니다.
- 상하 회전각 커브는 선형이지만, 전체 범위를 다 사용하지는 않습니다.
- 0% 스로틀 출력시, 상하 회전각 보정 설정은 거의 500 (0.05) 입니다.
- 최대 스로틀 출력시, 상하 회전각 보정 설정은 4500 (0.45) 밖에 안됩니다.
- 이 헬리콥터 형식에 더 큰 값을 사용하면 날개의 기능을 상실합니다.
- 이 헬리콥터의 경사판 서보는 0, 140에 220도 기울기를 가집니다.
- 서보 암 길이는 동일하지 않습니다.
- 두번째 세번째 서보는 긴 암을 가지나 처음 서보에 비해 1.3054배의 길이를 지닙니다.
- 서보 출력은 기계적 제약사항이 있어 -8000에서 8000 까지로 제한합니다.

#### 수직 이착륙 믹서 {#vtol_mixer}

수직 이착륙 시스템은 멀티로터 출력 목적의 [멀티로터 믹서](#multirotor_mixer)를 활용하며, 고정익 액츄에이터 동작을 위해 [결합 믹서](#summing_mixer)를 채용했습니다(그리고 틸트로터 수직 이착륙기의 경우 틸팅 서보를 결합 믹서에 붙입니다).

수직 이착륙기 믹서 시스템은 단일 믹서로 결합할 수 있는데, 액츄에이터는 입출력 포트 또는 FMU 포트로 연결하거나, 제각각의 믹서 파일로 입출력과 AUX용포트를 따로 나눕니다. 액츄에이터를 별도로 분리할 경우, 모든 멀티콥터 모터를 하나의 포트에 모아 붙이고, 모든 서보와 고정익 모터를 다른 포트에 모아 붙이는 방안을 추천드립니다.

> FMU 출력은 PX4 v1.11 부터 시작하는 멀티로터 모터에 활용할 수 있습니다. FMU 출력을 활용하려면 [VT_MC_ON_FMU=1](../advanced/parameter_reference.md#VT_MC_ON_FMU) 값을 설정하십시오(그렇지 않으면 고정익 비행 모드일 때 끌 수 없습니다).