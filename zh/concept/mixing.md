# 混控器和执行器

The PX4 architecture ensures that the airframe layout does not require special case handling in the core controllers.

Mixing means to take force commands (e.g. `turn right`) and translate them to actuator commands which control motors or servos. For a plane with one servo per aileron this means to command one of them high and the other low. The same applies for multicopters: Pitching forward requires changing the speed of all motors.

Separating the mixer logic from the actual attitude controller greatly improves reusability.

## 控制通道

A particular controller sends a particular normalized force or torque demand (scaled from -1..+1) to the mixer, which then sets individual actuators accordingly. The output driver (e.g. UART, UAVCAN or PWM) then scales it to the actuators native units, e.g. a PWM value of 1300.

{% mermaid %} graph LR; att_ctrl[Attitude Controller] --> act_group0[Actuator Control Group 0] gimbal_ctrl[Gimbal Controller] --> act_group2[Actuator Control Group 2] act_group0 --> output_group5[Actuator 5] act_group0 --> output_group6[Actuator 6] act_group2[Actuator Control Group 2] --> output_group0[Actuator 5] {% endmermaid %}

## 控制组

PX4 uses control groups (inputs) and output groups. Conceptually they are very simple: A control group is e.g. `attitude`, for the core flight controls, or `gimbal` for payload. An output group is one physical bus, e.g. the first 8 PWM outputs for servos. Each of these groups has 8 normalized (-1..+1) command ports, which can be mapped and scaled through the mixer. A mixer defines how each of these 8 signals of the controls are connected to the 8 outputs.

For a simple plane control 0 (roll) is connected straight to output 0 (aileron). For a multicopter things are a bit different: control 0 (roll) is connected to all four motors and combined with throttle.

### Control Group #0 (Flight Control)

* 0: roll (-1..1)
* 1: pitch (-1..1)
* 2: yaw (-1..1)
* 3: throttle (0..1 normal range, -1..1 for variable pitch / thrust reversers)
* 4: flaps (-1..1)
* 5: spoilers (-1..1)
* 6: airbrakes (-1..1)
* 7: landing gear (-1..1)

### Control Group #1 (Flight Control VTOL/Alternate)

* 0: roll ALT (-1..1)
* 1: pitch ALT (-1..1)
* 2: yaw ALT (-1..1)
* 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
* 4: reserved / aux0
* 5: reserved / aux1
* 6: reserved / aux2
* 7: reserved / aux3

### Control Group #2 (Gimbal)

* 0: gimbal roll
* 1: gimbal pitch
* 2: gimbal yaw
* 3: gimbal shutter
* 4: reserved
* 5: reserved
* 6: reserved
* 7: reserved (parachute, -1..1)

### Control Group #3 (Manual Passthrough)

* 0: RC roll
* 1: RC pitch
* 2: RC yaw
* 3: RC throttle
* 4: RC mode switch
* 5: RC aux1
* 6: RC aux2
* 7: RC aux3

### Control Group #6 (First Payload)

* 0: function 0 (default: parachute)
* 1: function 1
* 2: function 2
* 3: function 3
* 4: function 4
* 5: function 5
* 6: function 6
* 7: function 7

## 虚拟控制组

These groups are NOT mixer inputs, but serve as meta-channels to feed fixed wing and multicopter controller outputs into the VTOL governor module.

### Control Group #4 (Flight Control MC VIRTUAL)

* 0: roll ALT (-1..1)
* 1: pitch ALT (-1..1)
* 2: yaw ALT (-1..1)
* 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
* 4: reserved / aux0
* 5: reserved / aux1
* 6: reserved / aux2
* 7: reserved / aux3

### Control Group #5 (Flight Control FW VIRTUAL)

* 0: roll ALT (-1..1)
* 1: pitch ALT (-1..1)
* 2: yaw ALT (-1..1)
* 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
* 4: reserved / aux0
* 5: reserved / aux1
* 6: reserved / aux2
* 7: reserved / aux3

## 映射

由于同时存在多个控制组（比如说飞行控制、载荷等）和多个输出组（最开始 8 个 PWM 端口， UAVCAN 等），一个控制组可以向多个输出组发送指令。

{% mermaid %} graph TD; actuator_group_0-->output_group_5 actuator_group_0-->output_group_6 actuator_group_1-->output_group_0 {% endmermaid %}

## PX4 混控器定义

**ROMFS/px4fmu_common/mixers** 文件夹下的文件定义了在预定义的机架中可以使用的所有混控器。 这些文件可以作为建立自定义混控器的基础，或者用于一般的测试目的。

### 混控器描述文件命名

混控器如果负责混合 MAIN 输出端口的指令那么它的描述文件必须以 **XXXX.*main*.mix** 的形式进行命名，反之若其负责 AUX 输出则应该以 **XXXX.*aux*.mix** 的形式进行命名。

### 语法

混控器使用文本文件进行定义，文件中以单个大写字母后接一个冒号为开头的行是有意义的内容。 文件中其他类型的行都会被忽略，这就意味着你可以自有地在混控器描述文件中添加注释。

每一个文件可以定义多个混控器，混控器以何种形式分配给启动器完全取决于读取混控器定义文件的设备，混控器生成的执行器输出的数量则完全取决于混控器本身。

例如：每一个简单的或者空的混控器都会根据它在混控器描述文件中的出现顺序依次分配给输出 1 至 x 。

混控器定义以如下形式的行作为开头：

    <tag>: <mixer arguments>
    

上一行中的 tag 标签用于设定混控器类型：例如， 'M' 表示简单的求和混控器， 'R' 表示一个多旋翼的混控器。

#### 空的混控器（Null）

一个空的混控器不需要任何控制输入，并始终生成一个值为零的执行器输出。 通常情况下在一个混控器几何中使用空的混控器作为占位符号，以实现某种特定的执行器输出模式。

空的混控器使用如下形式定义：

    Z:
    

#### 一个简单的混控器

一个简单的混控器会将零个或者多个控制输入组合成一个执行器输出。 控制输入首先会被缩放，然后混合函数在进行输出缩放时会对结果进行求和。

一个简单的混控器的定义的开头如下：

    M: <control count>
    O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

如果 `&lt;control count&gt;` 为零，那么计算的结果也为零，混控器将输出 `&lt;offset&gt;` 这一固定值，该值的取值范围受 `&lt;lower limit&gt;` 和 `&lt;upper limit&gt;` 的限制。

上面的第二行还使用在之前讨论中提到的缩放参数对输出缩放器进行了定义。 同时，结果的计算是以浮点计算的形式进行的，在混控器定义文件中的值都将缩小 10000 倍，比如：实际中 -0.5 的偏移量（offset）在定义文件中保存为 -5000 。

定义文件将持续进行 `&lt;control count&gt;` 次定义，并以如下形式完成对各个控制输入量机器相应的缩放因子的描述：

    S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

> **Note** `S:` l行必须处于 `O:` 的下面。

`&lt;group&gt;` 参数指定了缩放器从哪个控制组中读取数据，而 `&lt;index&gt;` 参数则是定义了该控制组的偏移值。  
这些参数的设定值会随着读取混控器定义文件的设备的不同而发生改变。

当将混控器用于混合飞机的控制量时，编号为 0 的混控器组为飞机的姿态控制组，该控制组内编号 0 - 3 的选项通常分别便是滚转、俯仰、偏航和油门。

改行剩下的字段则是使用上文提及的缩放参数对控制量的缩放器进行了设定。 同时，结果的计算是以浮点计算的形式进行的，在混控器定义文件中的值都将缩小 10000 倍，比如：实际中 -0.5 的偏移量（offset）在定义文件中保存为 -5000 。

[这里](../airframes/adding_a_new_frame.md#mixer-file) 是一个典型混控器的示例文件。

#### 针对多旋翼的混控器

多旋翼的混控器将四组控制输入（俯仰、滚转、偏航和油门）整合到一组用于驱动电机转速控制器的执行器输出指令中。

该混控器使用如下形式的行进行定义：

    R: <geometry> <roll scale> <pitch scale> <yaw scale> <idlespeed>
    

支持的多旋翼类型为：

* 4x - X 构型的四旋翼
* 4+ - + 构型的四旋翼
* 6x - X 构型的六旋翼
* 6+ - + 构型的六旋翼
* 8x - X 构型的八旋翼
* 8+ - + 构型的八旋翼

滚转、俯仰和偏航的缩放因子大小都分别表示滚转、俯仰和边行控制相对于油门控制的比例。 同时，结果的计算是以浮点计算的形式进行的，在混控器定义文件中的值都将缩小 10000 倍，比如：实际中 0.5 的偏移量（offset）在定义文件中保存为 5000 。

滚转、俯仰和偏航输入量的范围应在 -1.0 到 1.0 之间，油门输入应该在 0.0 到 1.0 之间。每一个执行器的输出量应在 -1.0 到 1.0 之间。

怠速（Idlespeed）的设定值应在 0.0 到 1.0 之间。在这里怠速的值表示的是相对电机最大转速的百分比，当所有控制输入均为 0 的时候电机应在该转速下运行。

当有一个执行器出现饱和后，所有执行器的值都将被重新缩放以使得饱和执行器的输出被限制在 1.0 。

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

    S: <angle> <arm length> <scale> <offset> <lower limit> <upper limit>
    

The `<angle>` is in degrees, with 0 degrees being in the direction of the nose. Viewed from above, a positive angle is clock-wise. The `<arm length>` is a normalized length with 10000 being equal to 1. If all servo-arms are the same length, the values should al be 10000. A bigger arm length reduces the amount of servo deflection and a shorter arm will increase the servo deflection.

The servo output is scaled by `<scale> / 10000`. After the scaling, the `<offset>` is applied, which should be between -10000 and +10000. The `<lower limit>` and `<upper limit>` should be -10000 and +10000 for full servo range.

The tail rotor can be controller by adding a [simple mixer](#simple-mixer):

    M: 1
    S: 0 2  10000  10000      0 -10000  10000
    

By doing so, the tail rotor setting is directly mapped to the yaw command. This works for both servo-controlled tail-rotors, as well as for tail rotors with a dedicated motor.

The [blade 130 helicopter mixer](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/blade130.main.mix) can be viewed as an example. The throttle-curve starts with a slightly steeper slope to reach 6000 (0.6) at 50% thrust. It continues with a less steep slope to reach 10000 (1.0) at 100% thrust. The pitch-curve is linear, but does not use the entire range. At 0% throttle, the collective pitch setting is already at 500 (0.05). At maximum throttle, the collective pitch is only 4500 (0.45). Using higher values for this type of helicopter would stall the blades. The swash-plate servos for this helicopter are located at angles of 0, 140 and 220 degrees. The servo arm-lenghts are not equal. The second and third servo have a longer arm, by a ratio of 1.3054 compared to the first servo. The servos are limited at -8000 and 8000 because they are mechanically constrained.