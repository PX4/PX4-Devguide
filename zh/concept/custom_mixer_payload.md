# 自定义有效载荷混控器

此主题展示了如何添加一个用以可编程控制自定义载荷的自定义的[混控器](../concept/mixing.md)（比如电磁控制的机械手）

此主题是为那些想要支持现在没有现成控制组定义的载荷的开发者（比如说，云台有一个现成的控制组，但是机械手没有） 您应该已经阅读过[混控和执行器](../concept/mixing.md)


## 载荷混控器示例

有效载荷混控器只是一个 [简单混控器](../concept/mixing.md#simple-mixer) 用于映射从[控制组 #6 (第一个载荷)](../concept/mixing.md#control_group_6) 到某一特定输出的任一函数值 随后您可以将 uORB 主题发布到选定的控制组函数中，其值将被映射到指定的输出。

在这个例子中，我们将创建一个基于*遥控信号直穿的混控器*（[穿过辅助混控器](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/pass.aux.mix)） 这个混控器通常被加载到大型多旋翼的 AUX PWM 端口)。 它将4个用户自定义的遥控信号值（使用[RC_MAP_AUXx/RC_MAP_FLAPS](../advanced/parameter_reference.md#RC_MAP_AUX1)参数）直传到4个AUX口作为PWM输出

```
# 输出1-4的手动直传混控器

# AUX1通道 (由 RC_MAP_AUX1 参数选择遥控器的通道)
M: 1
S: 3 5  10000  10000      0 -10000  10000

# AUX2 通道(由 RC_MAP_AUX2 参数选择遥控器的通道)
M: 1
S: 3 6  10000  10000      0 -10000  10000

# AUX3 通道(由 RC_MAP_AUX3 参数选择遥控器的通道)
M: 1
S: 3 7  10000  10000      0 -10000  10000

# 襟翼通道 (由 RC_MAP_FLAPS 参数选择遥控器的通道)
M: 1
S: 3 4  10000  10000      0 -10000  10000
```

> **注释**文件定义了一个4输出的[简单混控器](../concept/mixing.md#simple-mixer) - `M：1` 表示由一个控制输入定义的输出(以下的`S` 行内容)。 - `S：3`_`n`_ 表示输入的是<>th<> 输入的[控制组3(手动通过)](../concept/mixing.md#control-group-3-manual-passthrough)。 所以对于r `S: 3 5`  输入被称为"RC aux1" （这表示参数`RC_MAP_AUX1`映射到遥控器通道） - The section declaration order defines the order of the outputs when assigned to a physical bus (e.g. the third section might be assigned to AUX3).


Start by copying the mixer file and putting it onto the SD Card at: **/etc/mixers/pass.aux.mix** (see [Mixing and Actuators > Loading a Custom Mixer](../concept/mixing.md#loading_custom_mixer).

删除带有载荷控制组函数输入的第一个部分：
- 更改此项
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
