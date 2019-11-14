# 믹싱과 액추에이터

PX4 구조는 코어 컨트롤러에서 에어프레임 레이아웃이 특별한 케이스에 처리를 필요로 하지 않는 것을 보장합니다.

믹싱은 물리적 명령어 (예. `turn right`)를 받아들이고 그것을 모터 컨트롤이나 서보 컨트롤과 같은 액추에이터 명령어로 변환합니다. 에일러론당 하나의 서보를 가진 비행기의 경우 하나는 높게 다른 하나는 낮게 명령하는 것을 의미합니다. 멀티콥터에도 동일하게 적용됩니다. 앞으로 피칭하기 위해서는 모든 모터의 속도 변화가 필요합니다.

실제 자세 컨트롤러부터 믹서의 기능을 모듈화 시키는 것은 재사용성을 증가시킵니다.

## 파이프라인 컨트롤

특정 컨트롤러는 특정 정규화된 물리력이나 토크를 (-1..+1 로 스케일 됨) 믹서로 보내고, 그러면 각각의 액추에이터들이 설정됩니다. 출력 드라이버 (예. UART, UAVCAN 또는 PWM) 은 그것을 액추에이터의 기본 단위로 변환합니다 (예. 1300의 PWM 값).

{% mermaid %} graph LR; att_ctrl[Attitude Controller] --> act_group0[Actuator Control Group 0] gimbal_ctrl[Gimbal Controller] --> act_group2[Actuator Control Group 2] act_group0 --> output_group5[Actuator 5] act_group0 --> output_group6[Actuator 6] act_group2[Actuator Control Group 2] --> output_group0[Actuator 5] {% endmermaid %}

## 컨트롤 그룹

PX4는 컨트롤 그룹 (입력) 과 출력 그룹을 사용합니다. 개념은 아주 간단합니다: 예를 들어 중요 비행 컨트롤러에 대한 컨트롤 그룹은 `attitude`, 페이로드에 대한 그룹은 `gimbal` 입니다. 출력 그룹은 하나의 물리적인 버스입니다 (예. 서보로의 첫 8개의 PWM 출력). 이들 각 그룹에는 믹서에 매핑되고 스케일될 수 있는 8개의 정규화된 (-1..+1) 명령 포트가 있습니다. 하나의 믹서는 어떻게 8개의 제어 신호 각각을 8개의 출력으로 연결할지 정의합니다.

간단한 비행기를 예로 들면, 컨트롤 0 (rolle) 은 곧바로 출력 0 (aileron) 에 연결됩니다. 멀티콥터는 조금 다릅니다. 컨트롤 0 (roll) 은 4개의 모터에 모두 연결되고 스로틀과 결합합니다.

### 컨트롤 그룹 #0 (비행 제어)

* 0: roll (-1..1)
* 1: pitch (-1..1)
* 2: yaw (-1..1)
* 3: throttle (0..1 normal range, -1..1 for variable pitch / thrust reversers)
* 4: flaps (-1..1)
* 5: spoilers (-1..1)
* 6: airbrakes (-1..1)
* 7: landing gear (-1..1)

### 컨트롤 그룹 #1 (수직이착륙기 비행제어/Alternate)

* 0: roll ALT (-1..1)
* 1: pitch ALT (-1..1)
* 2: yaw ALT (-1..1)
* 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
* 4: reserved / aux0
* 5: reserved / aux1
* 6: reserved / aux2
* 7: reserved / aux3

### 컨트롤 그룹 #2 (Gimbal)

* 0: gimbal roll
* 1: gimbal pitch
* 2: gimbal yaw
* 3: gimbal shutter
* 4: reserved
* 5: reserved
* 6: reserved
* 7: reserved (parachute, -1..1)

### 컨트롤 그룹 #3 (Manual Passthrough)

* 0: RC roll
* 1: RC pitch
* 2: RC yaw
* 3: RC throttle
* 4: RC mode switch
* 5: RC aux1
* 6: RC aux2
* 7: RC aux3

> **Note** 이 그룹은 오로지 RC 입력을 *normal operation* 동안에 특정한 출력으로 매핑하기 위해 사용됩니다 ( AUX2가 믹서에서 스케일링되는 예로[quad_x.maim.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/quad_x.main.mix#L7)를 참고하세요). 수동 입출력 페일세이프 (PX4FMU가 PX4IO 보드와의 통신을 멈춘경우) 이벤트에서는 컨트롤 그룹 0 입력에 의해 정의되고 매핑된 roll, pitch, yaw, throttle을 우선시 합니다 (다른 매핑들은 무시됨).

### 컨트롤 그룹 #6 (첫번째 페이로드)

* 0: function 0 (default: parachute)
* 1: function 1
* 2: function 2
* 3: function 3
* 4: function 4
* 5: function 5
* 6: function 6
* 7: function 7

## 가상 컨트롤 그룹

이 그룹들은 믹서의 입력들은 아니지만 고정익과 멀티콥터 컨트롤러의 출력을 VTOL govenor 모듈에 피드하기 위한 메타 채널을 제공합니다.

### 컨트롤 그룹 #4 (비행 제어 MC VIRTUAL)

* 0: roll ALT (-1..1)
* 1: pitch ALT (-1..1)
* 2: yaw ALT (-1..1)
* 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
* 4: reserved / aux0
* 5: reserved / aux1
* 6: reserved / aux2
* 7: reserved / aux3

### 컨트롤 그룹 #5 (비행 제어 FW VIRTUAL)

* 0: roll ALT (-1..1)
* 1: pitch ALT (-1..1)
* 2: yaw ALT (-1..1)
* 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
* 4: reserved / aux0
* 5: reserved / aux1
* 6: reserved / aux2
* 7: reserved / aux3

## 출력 그룹/매핑

하나의 출력그룹은 믹서에 매핑되고 스케일링 될 수있는 보통 8개의 정규화된 (-1..+1) 명령 포트를 가진 물리적인 버스입니다 (예. FMU PWM 출력, IO PWM 출력, UAVCAN 등).

믹서 파일은 출력이 적용되는 실제 *output group* (물리적인 버스) 를 명시적으로 정의하지는 않습니다. 대신에, 믹서의 목적은 (예. MAIN 또는 AUX 출력 컨트롤) [filename](#mixer_file_names)에서 알 수 있고, [startup scripts](../concept/system_startup.md) 에서 적절한 물리적인 버스로 매핑됩니다 ([rc.interface](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rc.interface) 에서 특정지어짐).

> **Note** MAIN 출력을 위해 사용되는 물리적인 버스가 항상 동일하지 않기 때문에 이 방식이 필요합니다. 물리적인 버스는 IO 보드를 가진 비행 컨트롤러에 의존하거나 모터 컨트롤을 위해 UAVCAN을 사용합니다. ([PX4 Reference Flight Controller Design > Main/IO Function Breakdown](../hardware/reference_design.md#mainio-function-breakdown)을 참고하세요) 스타트업 스크립트는 추상화된 디바이스를 이용하여 믹서파일을 보드에 적절한 디바이스 드라이버로 로드합니다. UAVCAN이 활성화 되어있으면, 메인 믹서는 `/dev/uavcan/esc`에 로드됩니다. 그렇지 않으면 `/dev/pwm_output0`에 로드됩니다. (이 장치는 IO 보드의 컨트롤러의 IO 드라이버에 매핑됩니다. 그렇지 않은 보드들은 FMU 드라이버에 매핑됩니다) Aux 믹서 파일은 IO 보드를 가진 픽스호크의 FMU 드라이버를 연결하는 `/dev/pwm_output1` 장치에 로드됩니다.

여러개의 컨트롤 그룹과 (비행 컨트롤, 페이로드 등) 출력 그룹 (버스들) 이 있기 때문에, 하나의 컨트롤 그룹은 여러개의 출력 그룹에게 명령어를 보낼 수 있습니다.

{% mermaid %} graph TD; actuator_group_0-->output_group_5 actuator_group_0-->output_group_6 actuator_group_1-->output_group_0 {% endmermaid %}

> **Note** 실제로는 스타트업 스크립트만이 믹서를 하나의 장치에 로드합니다. (output group) 이것은 기술적인 한계라기보다는 설정입니다. 예를 들어 메인 믹서를 여러개의 드라이버에 로드할 수 있고, UAVCAN과 메인 핀에 같은 신호를 가질 수 있습니다.

## PX4 믹서 정의

**ROMFS/px4fmu_common/mixers** 파일은 사전에 전의된 에어프레임에 사용될 믹서를 구현합니다. 이 파일들은 커스터마이징이나 일반적인 테스트를 위한 기초적인 파일로 사용될 수 있습니다.

### 믹서 파일 이름 {#mixer_file_names}

믹서 파일이름은 MAIN 출력의 믹싱을 담당하고 있다면 **XXXX.*main*.mix**, AUX 출력을 믹싱하고 있다면 **XXXX.*aux*.mix**이 되어야합니다.

### Syntax

믹서의 정의들은 텍스트 파일입니다. 대문자로 시작하고 콜론 뒤에가 중요합니다. 다른 모든 라인은 무시되기 때문에 쉽게 설명과 정의를 혼합하여 쓸 수 있습니다.

각 파일은 하나 이상의 믹서를 정의합니다. 믹서를 액추에이터에 할당하는 것은 믹서의 정의에 따라 다르며 믹서에 의해 생성되는 액추에이터 출력의 수는 믹서에 따라 다릅니다.

예를 들어, 간단한 믹서나 null 믹서는 출력 1을 x에 파일에 적혀진 순서대로 할당합니다.

믹서는 다음과 같은 형태로 시작합니다.

    <tag>: <mixer arguments>
    

태그는 믹서의 타입을 선택합니다. 'M'은 단순한 더하기 믹서, 'R'은 멀티콥터용 믹서, 등이 있습니다.

#### Null 믹서

Null 믹서는 제어는 안하고, 항상 0인 액추에이터 출력을 생성합니다. 일반적으로 null 믹서는 믹서의 집합들 중에서 특정한 패턴의 액추에이터 출력을 만들기 위한 플레이스홀더로 사용됩니다.

Null 믹서의 정의 형태는 다음과 같습니다.

    Z:
    

#### 심플 믹서

심플 믹서는 0개 이상의 컨트롤 입력을 하나의 액추에이터 출력으로 결합합니다. 입력을 스케일링 되고, 믹싱 함수는 출력 스케일러에 적용하기 전에 그 결과를 더합니다.

심플 믹서의 정의 형태는 다음과 같습니다.

    M: <control count>
    O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

만약 `<control count>`이 0이라면, 합은 사실상 0이 되고 믹서는 `<lower limit>`와 `<upper limit>`에 의해 제한된 `<offset>`을 출력할 것입니다.

두번째 라인은 위에서 설명한 대로 출력 스케일러를 파라미터와 함께 정의합니다. 부동소수점 계산이 이뤄지고 있는 동안에는 파일에 저장된 값은 10000으로 스케일됩니다. 예를 들면, -0.5는 -5000이 됩니다.

컨트롤 입력과 스케일링을 설명하는`<control count>` 엔트리들의 정의가 다음과 같은 형태로 계속됩니다.

    S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

> **Note** `S:` 라인은 반드시 `O:` 아래에 위치해야 합니다.

`<group>` 값은 스케일러가 읽게될 컨트롤 그룹을 확인하고, `<index>`은 해당 그룹   
이내의 오프셋값을 나타냅니다. 이 값은 믹서의 정의를 읽는 장치에 따라 다릅니다.

기체 컨트롤 믹싱에 사용될 때, 0번 믹서 그룹은 기체 자세 컨트롤 그룹입니다. 그리고 인덱스 0에서 3까지의 값은 각각 roll, pitch, yaw, 추력입니다.

나머지 라인들은 위에서 설명한 대로 컨트롤 스케일러와 파라미터를 설정합니다. 부동소수점 계산이 이뤄지고 있는 동안에는 파일에 저장된 값은 10000으로 스케일됩니다. 예를 들면, -0.5는 -5000이 됩니다.

전형적인 믹서의 예는 [here](../airframes/adding_a_new_frame.md#mixer-file)를 참고하세요.

#### 멀티콥터 믹서

멀티콥터 믹서는 모터 스피드 컨트롤러를 조종하기위해 4개의 컨트롤 입력 (roll, pitch, yaw, thrust) 을 하나의 액추에이터 출력의 모음으로 결합합니다.

믹서의 정의는 다음 형식의 한 줄입니다.

    R: <geometry> <roll scale> <pitch scale> <yaw scale> <idlespeed>
    

지원하는 지오메트리는 다음과 같습니다.

* 4x - quadrotor in X configuration
* 4+ - quadrotor in + configuration
* 6x - hexacopter in X configuration
* 6+ - hexacopter in + configuration
* 8x - octocopter in X configuration
* 8+ - octocopter in + configuration

Roll, pitch, yaw의 스케일 값 각각은 추력 컨트롤에 대한 roll, pitch, yaw 컨트롤에 대한 스케일링을 결정합니다. 부동소수점 계산이 이뤄지고 있는 동안에는 파일에 저장된 값은 10000으로 스케일됩니다. 예를 들면, -0.5는 -5000이 됩니다.

Roll, pitch, yaw 입력의 범위는 -1.0에서 1.0일 것입니다. 반면에 추력의 입력 범위는 0.0에서 1.0입니다. 각 액추에이터에 대한 출력의 범위는 -1.0에서 1.0입니다.

Idle 스피드의 범위는 0.0에서 1.0입니다. Idle 스피드는 모터의 최고 스피드에 상대적인데, 최고스피드는 모든 컨트롤 입력이 0일대 회전하기 위해 명령을 받는 모터의 스피드입니다.

한 액추에이터가 중점적으로 활용되고 있다면, 모든 액추에이터의 값은 중점적으로 사용되는 액추에이터의 범위를 1.0으로 제한하기 위해 다시 스케일링됩니다.

#### 헬리콥터 믹서

헬리콥터 믹서는 3개의 컨트롤 입력 (roll, pitch, thrust)를 4개의 출력 (swash-plate servos and main motor ESC setting) 으로 결합합니다. 헬리콥터 믹서의 첫번째 출력은 메인모터의 스로틀 세팅을 위한것입니다. 이후의 출력들은 swash-plate 서보를 위한 것입니다. Tail-rotor는 심플 믹서를 하나 추가함으로써 제어할 수 있습니다.

추력 컨트롤 입력은 메인 모터 설정과 swash-plate를 위한 collective pitch에 활용됩니다. Throttle-curve와 pitch-curve를 사용하며 5개의 포인트로 구성되어 있습니다.

> **Note** throttle-curve와 pitch-curve는 추력 스틱의 입력 위치를 스로틀와 pitch 값으로 각각 매핑합니다. 이를 통해 비행유형에 따라 비행 특성을 조정하는 것을 지원합니다. 커브를 조정하는 방법은 [this guide](https://www.rchelicopterfun.com/rc-helicopter-radios.html)에서 찾을 수 있습니다. *Programmable Throttle Curves* and *Programmable Pitch Curves*를 검색해보세요.

믹서의 정의는 다음과 같이 시작합니다.

    H: <number of swash-plate servos, either 3 or 4>
    T: <throttle setting at thrust: 0%> <25%> <50%> <75%> <100%>
    P: <collective pitch at thrust: 0%> <25%> <50%> <75%> <100%>
    

`T:` 는 스로틀-커브를 정의합니다. `P:` 는 피치-커브를 정의합니다. 두 커브모두 0에서 10000범위의 5개 포인트를 포함합니다. 간단한 선형 동작을 위해서는 5개 포인트의 값은 다음처럼 되어야 합니다. `0 2500 5000 7500 10000`.

다음에는 swash-plate 서보 (3 또는 4) 를 위한 라인들이 다음과 같은 형태로 옵니다.

    S: <angle> <arm length> <scale> <offset> <lower limit> <upper limit>
    

`<angle>`은 진입 차수입니다. 0 degree는 노즈의 방향입니다. 위에서 봤듯이, 시계방향을 기준으로 합니다. `<arm length>`은 10000이 1로 정규화된 길이 입니다. 만약 모든 servo-arm이 같은 길이를 가졌다면, 그 값은 10000이 되야합니다. 더 큰 arm 길이는 서보 편차의 양을 작게만들고, 짧은 것은 크게만듭니다.

서보 출력은 `<scale> / 10000`으로 스케일됩니다. 스케일링 이후의 값든은 -10000에서 +10000 사이에 있어야 합니다. `<lower limit>` 과 `<upper limit>` 모든 서보 범위를 커버하기 위해 -10000에서 +10000이 되어야 합니다.

Tail rote는 [simple mixer](#simple-mixer)를 하나 추가함으로써 제어할 수 있습니다.

    M: 1
    S: 0 2  10000  10000      0 -10000  10000
    

이렇게함으로써, tail rotor의 설정은 yaw 명령어에 직접적으로 전달됩니다. 이것은 서보가 컨트롤하는 tail rotor와 dedicated motor용 tail rotor 둘다에게 동작합니다.

[blade 130 helicopter mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/blade130.main.mix)를 하나의 예로 볼 수 있습니다. 스로틀-커브는 50%의 추력에 6000 (0.6) slope를 달성할 수 있도록 약간 가파른 경사로 시작합니다. 100 % 추력에서 10000 (1.0) 에 도달하기 위해 가파른 경사가 계속됩니다. 피치-커브는 직선입니다. 하지만 모든 범위를 다 사용하지는 않습니다. 0% 스로틀에서, collective pitch 설정은 이미 500 (0.05) 입니다. 최대의 스로틀에서, collective pitch는 4500 (0.45) 입니다. 이 타입의 헬리콥터에 높은 값을 사용하면 날개가 멈출 것입니다. 헵리콥터를 위한 swash-plate 서보는 0, 140 그리고 220의 각도에 위치합니다. Servo arm 길이는 동일하지 않습니다. 두번째와 세번째 서보는 첫번째에 비행 비율적으로 1.3054 긴 arm을 갖고 있습니다. Servo는 기계적으로 제약이 있어서 -8000에서 8000으로 제한됩니다