# 개별 페이로드 믹서

이 주제에서는 개별 페이로드를 프로그램으로 제어하는 개별 [믹서](../concept/mixing.md) 추가 방법을 다룹니다(예: 전자기 집게).

기존 제어 분류 정의에 없는 페이로드 형식을 지원하려는 개발자에게 이 주제를 안내하고자합니다(예: 짐벌은 제어 그룹이 있으나, 집게는 그렇지 않음). [믹싱과 액츄에이터](../concept/mixing.md)를 우선 읽고 오셔야 합니다.


## 페이로드 믹서 예제

A payload mixer is just a [summing mixer](../concept/mixing.md#summing_mixer) that maps any of the function values from [Control Group #6 (First Payload)](../concept/mixing.md#control_group_6) to a particular output. You can then publish uORB topics to the selected control group function and their value will be mapped to the specified output.

For this example, we'll create a custom mixer based on the *RC passthrough mixer* ([pass.aux.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/pass.aux.mix)). This mixer is commonly loaded into the AUX PWM ports on large multicopters). It passes through the values of 4 user-defined RC channels (set using the [RC_MAP_AUXx/RC_MAP_FLAPS](../advanced/parameter_reference.md#RC_MAP_AUX1) parameters) to the first four outputs on the AUX PWM output.

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

> **Note** The file defines four [summing mixers](../concept/mixing.md#summing_mixer) (for four outputs). - `M: 1` indicates an output that is defined by one control input (the following `S` line). - `S: 3`_`n`_ indicates that the input is the n<>th<> input of [Control Group 3 (Manual Passthrough)](../concept/mixing.md#control-group-3-manual-passthrough). So for `S: 3 5` the input is called "RC aux1" (this maps to the RC channel set in parameter `RC_MAP_AUX1`). - The section declaration order defines the order of the outputs when assigned to a physical bus (e.g. the third section might be assigned to AUX3).


Start by copying the mixer file and putting it onto the SD Card at: **/etc/mixers/pass.aux.mix** (see [Mixing and Actuators > Loading a Custom Mixer](../concept/mixing.md#loading_custom_mixer).

Remove the first section with a payload control group function input:
- Change this:
  ```
  # AUX1 channel (control group 3, RC CH5) (select RC channel with RC_MAP_AUX1 param)
  M: 1
  S: 3 5  10000  10000      0 -10000  10000
  ```
- To:
  ```
  # Payload 1 (control group 6) channel 1
  M: 1
  S: 6 1  10000  10000      0 -10000  10000
  ```

Because this output is in the first position in the file it will map to the first AUX PWM output (unless UAVCAN is enabled). This output will now respect updates to the payload control group (6) output 1.

Control group 6 will need to be defined in the code as well (it is missing!):
- Add `actuator_controls_6` to the TOPICS definition in [/msg/actuator_controls.msg](https://github.com/PX4/Firmware/blob/master/msg/actuator_controls.msg#L17):
  ```
  # TOPICS actuator_controls actuator_controls_0 actuator_controls_1 actuator_controls_2 actuator_controls_3 actuator_controls_6
  ```
- Increase `NUM_ACTUATOR_CONTROL_GROUPS` to 7 in the same file.
- Subscribe to the additional control group in the output library ([/src/lib/mixer_module/mixer_module.cpp#L52](https://github.com/PX4/Firmware/blob/master/src/lib/mixer_module/mixer_module.cpp#L52)) in the `MixingOutput` constructor. It should look like this:
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

Putting an output on group 6 works by publishing actuator control group 6. First you have to create the publication. This should happen once when the PX4 module is initialized (look for places where this pattern is already being used):
```
uORB::Publication<actuator_controls_s> _actuator_controls_pub{ORB_ID(actuator_controls_6)};
```

Then you need to publish the first message:
```
actuator_controls_s _act_controls{};
_act_controls.timestamp = hrt_absolute_time();
// set the first output to 50% positive (this would rotate a servo halfway into one of its directions)
_act_controls.control[0] = 0.5f;
_actuator_controls_pub.publish(_act_controls);
```
