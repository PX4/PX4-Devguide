# 混控器和执行器

PX4 的系统构架可确保不需要在核心控制器中对不同的机身布局进行任何特殊的处理。

混合意味着接收力的指令（比如： `向右转`），然后将这些指令转换成实际的执行器指令来控制电机或者舵机。 对于一个每片副翼都有一个舵机的飞机而言这就意味着控制这两个舵机一个向上偏转，一个向下偏转。 这也适用于多旋翼：向前俯仰需要改变所有电机的转速。

将混控逻辑与实际的姿态控制器分离开来大大提高了程序的可复用性。

## 控制通道

特定的控制器发送一个特定的归一化的力或力矩指令（缩放至 -1..+1 ）给混控器，混控器则相应地去设置每个单独的执行器。 控制量输出驱动程序（比如：UART, UAVCAN 或者 PWM）则将混控器的输出所放为执行器实际运行时的原生单位， 例如输出一个值为 1300 的 PWM 指令。

{% mermaid %} graph LR; att_ctrl[Attitude Controller] --> act_group0[Actuator Control Group 0] gimbal_ctrl[Gimbal Controller] --> act_group2[Actuator Control Group 2] act_group0 --> output_group5[Actuator 5] act_group0 --> output_group6[Actuator 6] act_group2[Actuator Control Group 2] --> output_group0[Actuator 5] {% endmermaid %}

## 控制组

PX4 系统中使用控制组（输入）和输出组。 从概念上讲这两个东西非常简单： 一个控制组可以是核心飞行控制器的 `姿态`，也可以是载荷的 `云台` 。 一个输出组则是一个物理上的总线，例如 飞控上最开始的 8 个 PWM 舵机输出口。 每一个组都有 8 个单位化（-1..+1）的指令端口，这些端口可以通过混控器进行映射和缩放。 混控器定义了这 8 个控制信号如何连接至 8 个输出口。

对于一个简单的飞机来说 control 0（滚转）直接与 output 0（副翼）相连接。 对于多旋翼而言事情要稍有不同：control 0（滚转）与全部四个电机相连接，并会被整合至油门指令中。

### 控制组 #0 (Flight Control)

* 0：roll (-1..1)
* 1：pitch (-1..1)
* 2：yaw (-1..1)
* 3：throttle （正常范围为 0..1，变距螺旋桨和反推动力情况下范围为 -1..1）
* 4：flaps (-1..1)
* 5：spoilers (-1..1)
* 6：airbrakes (-1..1)
* 7：landing gear (-1..1)

### 控制组 #1 (Flight Control VTOL/Alternate)

* 0：roll ALT (-1..1)
* 1：pitch ALT (-1..1)
* 2：yaw ALT (-1..1)
* 3：throttle ALT （正常范围为 0..1，变距螺旋桨和反推动力情况下范围为 -1..1）
* 4：保留 / aux0
* 5：reserved / aux1
* 6：保留 / aux2
* 7：保留 / aux3

### 控制组 #2 （Gimbal）

* 0：gimbal roll
* 1：gimbal pitch
* 2: gimbal yaw
* 3: gimbal shutter
* 4：保留
* 5：保留
* 6：保留
* 7：保留 (降落伞, -1..1)

### 控制组 #3 (Manual Passthrough)

* 0: RC roll
* 1: RC pitch
* 2: RC yaw
* 3: RC throttle
* 4: RC mode switch
* 5: RC aux1
* 6: RC aux2
* 7: RC aux3

> **备注** 此组仅用于定义在 *普通操作* 期间的 RC输入的映射到具体输出(见 [quad_x.main.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/quad_x.main.mix#L7) 关于AUX2在混合器中缩放的示例) In the event of manual IO failsafe override (if the PX4FMU stops communicating with the PX4IO board) only the mapping/mixing defined by control group 0 inputs for roll, pitch, yaw and throttle are used (other mappings are ignored).

### 控制组 #6 (First Payload)

* 0: function 0 (默认：降落伞)
* 1: function 1
* 2: function 2
* 3: function 3
* 4: function 4
* 5: function 5
* 6: function 6
* 7: function 7

## 虚拟控制组

虚拟控制组并不作为混控器的输入量使用，它们将作为元通道（meta-channels）将固定翼控制器和多旋翼控制器的输出传递给 VOTL 调节器模块（VTOL governor module）。

### 控制组 #4 (Flight Control MC VIRTUAL)

* 0: roll ALT (-1..1)
* 1: pitch ALT (-1..1)
* 2: yaw ALT (-1..1)
* 3: throttle ALT （正常范围为 0..1，变距螺旋桨和反推动力情况下范围为 -1..1）
* 4：保留 / aux0
* 5：保留 / aux1
* 6：保留 / aux2
* 7：保留 / aux3

### 控制组 #5 (Flight Control FW VIRTUAL)

* 0: roll ALT (-1..1)
* 1: pitch ALT (-1..1)
* 2: yaw ALT (-1..1)
* 3: throttle ALT （正常范围为 0..1，变距螺旋桨和反推动力情况下范围为 -1..1）
* 4：保留 / aux0
* 5：保留 / aux1
* 6：保留 / aux2
* 7：保留 / aux3

## Output Groups/Mapping

An output group is one physical bus (e.g. FMU PWM outputs, IO PWM outputs, UAVCAN etc.) that has N (usually 8) normalized (-1..+1) command ports that can be mapped and scaled through the mixer.

The mixer file does not explicitly define the actual *output group* (physical bus) where the outputs are applied. Instead, the purpose of the mixer (e.g. to control MAIN or AUX outputs) is inferred from the mixer [filename](#mixer_file_names), and mapped to the appropriate physical bus in the system [startup scripts](../concept/system_startup.md) (and in particular in [rc.interface](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rc.interface)).

> **Note** This approach is needed because the physical bus used for MAIN outputs is not always the same; it depends on whether or not the flight controller has an IO Board (see [PX4 Reference Flight Controller Design > Main/IO Function Breakdown](../hardware/reference_design.md#mainio-function-breakdown)) or uses UAVCAN for motor control. The startup scripts load the mixer files into the appropriate device driver for the board, using the abstraction of a "device". The main mixer is loaded into device `/dev/uavcan/esc` (uavcan) if UAVCAN is enabled, and otherwise `/dev/pwm_output0` (this device is mapped to the IO driver on controllers with an I/O board, and the FMU driver on boards that don't). The aux mixer file is loaded into device `/dev/pwm_output1`, which maps to the FMU driver on Pixhawk controllers that have an I/O board.

Since there are multiple control groups (like flight controls, payload, etc.) and multiple output groups (busses), one control group can send commands to multiple output groups.

{% mermaid %} graph TD; actuator_group_0-->output_group_5 actuator_group_0-->output_group_6 actuator_group_1-->output_group_0 {% endmermaid %}

> **Note** In practice, the startup scripts only load mixers into a single device (output group). This is a configuration rather than technical limitation; you could load the main mixer into multiple drivers and have, for example, the same signal on both UAVCAN and the main pins.

## PX4 混控器定义

Files in **ROMFS/px4fmu_common/mixers** implement mixers that are used for predefined airframes. They can be used as a basis for customisation, or for general testing purposes.

### Mixer File Names {#mixer_file_names}

A mixer file must be named **XXXX.*main*.mix** if it is responsible for the mixing of MAIN outputs or **XXXX.*aux*.mix** if it mixes AUX outputs.

### 语法

Mixer definitions are text files; lines beginning with a single capital letter followed by a colon are significant. All other lines are ignored, meaning that explanatory text can be freely mixed with the definitions.

Each file may define more than one mixer; the allocation of mixers to actuators is specific to the device reading the mixer definition, and the number of actuator outputs generated by a mixer is specific to the mixer.

For example: each simple or null mixer is assigned to outputs 1 to x in the order they appear in the mixer file.

A mixer begins with a line of the form

    <tag>: <mixer arguments>
    

The tag selects the mixer type; 'M' for a simple summing mixer, 'R' for a multirotor mixer, etc.

#### 空的混控器（Null）

A null mixer consumes no controls and generates a single actuator output whose value is always zero. Typically a null mixer is used as a placeholder in a collection of mixers in order to achieve a specific pattern of actuator outputs.

The null mixer definition has the form:

    Z:
    

#### 一个简单的混控器

A simple mixer combines zero or more control inputs into a single actuator output. Inputs are scaled, and the mixing function sums the result before applying an output scaler.

A simple mixer definition begins with:

    M: <control count>
    O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

If `<control count>` is zero, the sum is effectively zero and the mixer will output a fixed value that is `<offset>` constrained by `<lower limit>` and `<upper limit>`.

The second line defines the output scaler with scaler parameters as discussed above. Whilst the calculations are performed as floating-point operations, the values stored in the definition file are scaled by a factor of 10000; i.e. an offset of -0.5 is encoded as -5000.

The definition continues with `<control count>` entries describing the control inputs and their scaling, in the form:

    S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

> **Note** The `S:` lines must be below the `O:` line.

The `<group>` value identifies the control group from which the scaler will read, and the `<index>` value an offset within that group.  
These values are specific to the device reading the mixer definition.

When used to mix vehicle controls, mixer group zero is the vehicle attitude control group, and index values zero through three are normally roll, pitch, yaw and thrust respectively.

The remaining fields on the line configure the control scaler with parameters as discussed above. Whilst the calculations are performed as floating-point operations, the values stored in the definition file are scaled by a factor of 10000; i.e. an offset of -0.5 is encoded as -5000.

An example of a typical mixer file is explained [here](../airframes/adding_a_new_frame.md#mixer-file).

#### 针对多旋翼的混控器

The multirotor mixer combines four control inputs (roll, pitch, yaw, thrust) into a set of actuator outputs intended to drive motor speed controllers.

The mixer definition is a single line of the form:

    R: <geometry> <roll scale> <pitch scale> <yaw scale> <idlespeed>
    

The supported geometries include:

* 4x - X 构型的四旋翼
* 4+ - + 构型的四旋翼
* 6x - X 构型的六旋翼
* 6+ - + 构型的六旋翼
* 8x - X 构型的八旋翼
* 8+ - + 构型的八旋翼

Each of the roll, pitch and yaw scale values determine scaling of the roll, pitch and yaw controls relative to the thrust control. Whilst the calculations are performed as floating-point operations, the values stored in the definition file are scaled by a factor of 10000; i.e. an factor of 0.5 is encoded as 5000.

Roll, pitch and yaw inputs are expected to range from -1.0 to 1.0, whilst the thrust input ranges from 0.0 to 1.0. Output for each actuator is in the range -1.0 to 1.0.

Idlespeed can range from 0.0 to 1.0. Idlespeed is relative to the maximum speed of motors and it is the speed at which the motors are commanded to rotate when all control inputs are zero.

In the case where an actuator saturates, all actuator values are rescaled so that the saturating actuator is limited to 1.0.

#### 针对直升机的混控器

The helicopter mixer combines three control inputs (roll, pitch, thrust) into four outputs ( swash-plate servos and main motor ESC setting). The first output of the helicopter mixer is the throttle setting for the main motor. The subsequent outputs are the swash-plate servos. The tail-rotor can be controlled by adding a simple mixer.

The thrust control input is used for both the main motor setting as well as the collective pitch for the swash-plate. It uses a throttle-curve and a pitch-curve, both consisting of five points.

> **Note** The throttle- and pitch- curves map the "thrust" stick input position to a throttle value and a pitch value (separately). This allows the flight characteristics to be tuned for different types of flying. An explanation of how curves might be tuned can be found in [this guide](https://www.rchelicopterfun.com/rc-helicopter-radios.html) (search on *Programmable Throttle Curves* and *Programmable Pitch Curves*).

The mixer definition begins with:

    H: <number of swash-plate servos, either 3 or 4>
    T: <throttle setting at thrust: 0%> <25%> <50%> <75%> <100%>
    P: <collective pitch at thrust: 0%> <25%> <50%> <75%> <100%>
    

`T:` defines the points for the throttle-curve. `P:` defines the points for the pitch-curve. Both curves contain five points in the range between 0 and 10000. For simple linear behavior, the five values for a curve should be `0 2500 5000 7500 10000`.

This is followed by lines for each of the swash-plate servos (either 3 or 4) in the following form:

    S: &lt;angle&gt; &lt;arm length&gt; &lt;scale&gt; &lt;offset&gt; &lt;lower limit&gt; &lt;upper limit&gt;
    

The `<angle>` is in degrees, with 0 degrees being in the direction of the nose. Viewed from above, a positive angle is clock-wise. The `<arm length>` is a normalized length with 10000 being equal to 1. If all servo-arms are the same length, the values should al be 10000. A bigger arm length reduces the amount of servo deflection and a shorter arm will increase the servo deflection.

The servo output is scaled by `<scale> / 10000`. After the scaling, the `<offset>` is applied, which should be between -10000 and +10000. The `<lower limit>` and `<upper limit>` should be -10000 and +10000 for full servo range.

The tail rotor can be controller by adding a [simple mixer](#simple-mixer):

    M: 1
    S: 0 2  10000  10000      0 -10000  10000
    

By doing so, the tail rotor setting is directly mapped to the yaw command. This works for both servo-controlled tail-rotors, as well as for tail rotors with a dedicated motor.

The [blade 130 helicopter mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/blade130.main.mix) can be viewed as an example. The throttle-curve starts with a slightly steeper slope to reach 6000 (0.6) at 50% thrust. It continues with a less steep slope to reach 10000 (1.0) at 100% thrust. The pitch-curve is linear, but does not use the entire range. At 0% throttle, the collective pitch setting is already at 500 (0.05). At maximum throttle, the collective pitch is only 4500 (0.45). Using higher values for this type of helicopter would stall the blades. The swash-plate servos for this helicopter are located at angles of 0, 140 and 220 degrees. The servo arm-lenghts are not equal. The second and third servo have a longer arm, by a ratio of 1.3054 compared to the first servo. The servos are limited at -8000 and 8000 because they are mechanically constrained.