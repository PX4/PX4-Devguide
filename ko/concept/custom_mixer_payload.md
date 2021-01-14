!REDIRECT "https://docs.px4.io/master/ko/concept/custom_mixer_payload.html"

# 개별 탑재 장치 믹서

이 주제에서는 개별 탑재 장치를 프로그램으로 제어하는 개별 [믹서](../concept/mixing.md) 추가 방법을 다룹니다(예: 전자기 집게).

기존 제어 분류 정의에 없는 탑재 장치 형식을 지원할 개발자에게 이 주제를 안내하고자합니다(예: 짐벌은 제어 그룹이 있으나, 집게는 그렇지 않음). [믹싱과 액츄에이터](../concept/mixing.md)를 우선 읽고 오셔야 합니다.


## 탑재 장치 믹서 예제

탑재 장치 믹서는 단지 [제어 분류 #6 (첫번째 탑재 장치)](../concept/mixing.md#control_group_6)로부터 각각의 출력으로의 기능 값에 대응하는 [믹서 결합](../concept/mixing.md#summing_mixer)을 수행할 뿐입니다. uORB 토픽을 지정 출력으로 대응할 선택 그룹 함수와 값으로 uORB 토픽을 내보낼 수 있습니다.

For this example, we'll create a custom mixer based on the *RC passthrough mixer* ([pass.aux.mix](https://github.com/PX4/PX4-Autopilot/blob/master/ROMFS/px4fmu_common/mixers/pass.aux.mix)). 이 믹서는 보통 대형 멀티콥터에서 AUX PWM 포트로 불러옵니다. 이 믹서로 4개의 사용자 정의 원격 조종 채널 값을 처리합니다([RC_MAP_AUXx/RC_MAP_FLAPS](../advanced/parameter_reference.md#RC_MAP_AUX1) 매개변수로 설정). 그리하여 첫번째 4개의 출력은 AUX PWM 출력입니다.

```
# Manual pass through mixer for servo outputs 1-4

# AUX1 channel (select RC channel with RC_MAP_AUX1 param)
M: 1
S: 3 5  10000  10000      0 -10000  10000

# AUX2 channel (select RC channel with RC_MAP_AUX2 param)
M: 1
S: 3 6  10000  10000      0 -10000  10000

# AUX3 channel (select RC channel with RC_MAP_AUX3 param)
M: 1
S: 3 7  10000  10000      0 -10000  10000

# FLAPS channel (select RC channel with RC_MAP_FLAPS param)
M: 1
S: 3 4  10000  10000      0 -10000  10000
```

> **Note** 파일에서는 4개의 (출력에 대한) [믹서 결합](../concept/mixing.md#summing_mixer)을 나타냅니다. - `M: 1` 하나의 제어 입력(다음 `S`행)에 대한 출력을 나타냅니다. - `S: 3`_`n`_ [제어 분류 3 (수동 처리)](../concept/mixing.md#control-group-3-manual-passthrough)의 n번째 입력을 나타냅니다. 따라서 `S: 3 5` 은 "원격 조종 AUX1"입니다(이 입력은 `RC_MAP_AUX1` 매개변수의 원격 조종 채널 세트에 대응합니다). 섹션 선언 순서는 물리 버스에 할당할 출력 순서를 정의합니다(예: 세번째 섹션은 AUX3에 할당함).


믹서 파일을 복사하여 **/etc/mixers/pass.aux.mix** SD 카드의  <0>/etc/mixers/pass.aux.mix</0> 디렉터리 위치에 놓는 과정으로 시작합니다([믹싱과 액츄에이터 > 개별 믹서 불러오기](../concept/mixing.md#loading_custom_mixer) 참고).

첫번째 섹션과 탑재 장치 제어 분류 함수 입력을 제거하십시오:
- 아래를:
  ```
  # AUX1 channel (control group 3, RC CH5) (select RC channel with RC_MAP_AUX1 param)
  M: 1
  S: 3 5  10000  10000      0 -10000  10000
  ```
- 다음으로 바꾸십시오:
  ```
  # Payload 1 (control group 6) channel 1
  M: 1
  S: 6 1  10000  10000      0 -10000  10000
  ```

이 출력은 파일의 처음에 있기 때문에 (UAVCAN을 활성화하기 전에는) 첫번째 AUX PWM 출력에 대응합니다. 이 출력은 앞으로 탑재 장치 제어 분류 (6)의 출력 1을 업데이트합니다.

제어 분류 6은 코드에서 정의한 그대로 필요합니다(빠져있음!):
- Add `actuator_controls_6` to the TOPICS definition in [/msg/actuator_controls.msg](https://github.com/PX4/PX4-Autopilot/blob/master/msg/actuator_controls.msg#L17):
  ```
  # TOPICS actuator_controls actuator_controls_0 actuator_controls_1 actuator_controls_2 actuator_controls_3 actuator_controls_6
  ```
- 동일한 파일에서 `NUM_ACTUATOR_CONTROL_GROUPS` 값을 7로 바꾸십시오.
- Subscribe to the additional control group in the output library ([/src/lib/mixer_module/mixer_module.cpp#L52](https://github.com/PX4/PX4-Autopilot/blob/master/src/lib/mixer_module/mixer_module.cpp#L52)) in the `MixingOutput` constructor. 대략 다음과 같습니다:
  ```
    {&interface, ORB_ID(actuator_controls_0)},
    {&interface, ORB_ID(actuator_controls_1)},
    {&interface, ORB_ID(actuator_controls_2)},
    {&interface, ORB_ID(actuator_controls_3)},
  ```
  ```
    {&interface, nullptr},
    {&interface, nullptr},
    {&interface, ORB_ID(actuator_controls_6)},
  ```

분류 6번의 출력은 액츄에이터 제어 분류 6번과 동작합니다. 우선 퍼블리케이션 객체를 만들어야합니다. 이 과정은 PX4 모듈을 초기화할 때 일어납니다(이 반복 규칙을 이미 사용한 곳을 살펴보십시오):
```
uORB::Publication<actuator_controls_s> _actuator_controls_pub{ORB_ID(actuator_controls_6)};
```

그 다음 첫 메시지를 내보내야합니다:
```
actuator_controls_s _act_controls{};
_act_controls.timestamp = hrt_absolute_time();
// set the first output to 50% positive (this would rotate a servo halfway into one of its directions)
_act_controls.control[0] = 0.5f;
_actuator_controls_pub.publish(_act_controls);
```
