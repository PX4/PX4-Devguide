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

> **备注** 此组仅用于定义在 *普通操作* 期间的 RC输入的映射到具体输出(见 [quad_x.main.mix](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/quad_x.main.mix#L7) 关于AUX2在混合器中缩放的示例) 在手动 IO 故障安全覆盖事件中(如果 PX4FMU 停止与 PX4IO 板通信)， 只有 pitch、yaw和 throttle 这些控制组0定义的映射/混合被使用 (忽略其他映射)。

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

## 输出组/映射

一个产出组是一个物理总线(例如 FMU PWM 输出，IO PWM 输出，UAVCAN 等)。 具有 N (通常为8) 个规范化(-1..+1)的命令端口，可以通过混音器映射和缩放。

混音器文件没有明确定义输出应用的实际 *输出组* (物理总线)。 相反，混合物的目的 (例如控制MAIN或 AUX 输出) 从混音器 [ filename ](#mixer_file_names) 中推断，并映射到系统中适当的物理总线 [startup scripts](../concept/system_startup.md) (尤其是[rc.interface](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rc.interface))。

> ** Note ** 这种方法很有必要，因为用于MAIN输出的物理总线并不总是一样的； 它取决于飞行控制器是否有 IO Board(见[PX4 Reference Flight Controller Design > Main/IO Function Breakdown](../hardware/reference_design.md#mainio-function-breakdown)) 或使用UAVCAN 进行发动机控制。 启动脚本使用"设备"抽象将混音器文件加载到板子适当的设备驱动器。 如果 UAVCAN 已启用，主混音器将被加载到设备`/dev/uavcan/esc` (uavcan) 否则`/dev/pwm_output0` (此设备已映射给具有I/O 板的控制器的 IO 驱动，且 FMU 驱动程序已映射到未映射的板上)。 Aux 混控器 文件被加载到设备 `/dev/pwm_output1`, 它将映射到 Pixhawk 控制器上拥有 I/O 板子的 FMU 驱动程序。

因为有多个控制组(例如飞行控制、有效载荷等)。 和多个输出组(总线) ，一个控制组可以向多个输出组发送命令。

graph TD; actuator_group_0-->output_group_5 actuator_group_0-->output_group_6 actuator_group_1-->output_group_0

> **Note** 在实践中，启动脚本只会加载混控器到单个设备 (输出组)。 这是一个配置而不是技术限制； 您可以将主混音器加载到多个驱动器中，例如在UAVCAN 和主引脚上都有相同的信号。

## PX4 混控器定义

**ROMFS/px4fmu_common/mixers**中的文件实现用于预定义机架的混音器。 它们可以用作定制或一般测试的基础。

### 混合文件名称 {#mixer_file_names}

混控器如果负责混合 MAIN 输出端口的指令那么它的描述文件必须以 **XXXX.*main*.mix** 的形式进行命名，若其负责 AUX 输出则应该以 **XXXX.*aux*.mix** 的形式进行命名。

### 语法

混控器使用文本文件进行定义，文件中以单个大写字母后接一个冒号为开头的行是有意义的内容。 所有其他行都被忽视，这意味着解释性案文可以自由地与定义混在一起。

每一个文件可以定义多个混控器，混控器以何种形式分配给启动器完全取决于读取混控器定义文件的设备，混控器生成的执行器输出的数量则完全取决于混控器本身。

例如：每一个简单的或者空的混控器都会根据它在混控器描述文件中的出现顺序依次分配给输出 1 至 x 。

混控器定义以如下形式的行作为开头：

    <tag>: <mixer arguments>
    

上一行中的 tag 标签用于设定混控器类型：例如， 'M' 表示简单的求和混控器， 'R' 表示一个多旋翼的混控器。

#### 空的混控器（Null）

一个空的混控器不需要任何控制输入，并始终生成一个值为零的执行器输出。 通常情况下在一个混控器集合中使用空的混控器作为占位符号，以实现某种特定的执行器输出模式。

空的混控器使用如下形式定义：

    Z:
    

#### 一个简单的混控器

一个简单的混合器将零或更多的控制输入并入一个单一的促动器输出。 控制输入首先会被缩放，然后混合函数在进行输出缩放时会对结果进行求和。

一个简单的混控器的定义的开头如下：

    M: <control count>
    O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

如果 `<control count>` 为零，那么计算的结果也为零，混控器将输出 `<offset>` 这一固定值，该值的取值范围受 `<lower limit>` 和 `<upper limit>` 的限制。

第二行用上文讨论的缩放参数定义了输出缩放器。 同时，结果的计算是以浮点计算的形式进行的，在混控器定义文件中的值都将扩大 10000 倍，比如：实际中 -0.5 的偏移量（offset）在定义文件中保存为 -5000 。

定义继续以 `<control count>` 条目描述控制输入及其缩放的形式：

    S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
    

> **Note** `S:` 行必须处于 `O:` 的下面。

`<group>` 指定了缩放器从哪个控制组中读取数据，而 `<index>` 则是定义了该控制组的偏移值。  
这些参数的设定值会随着读取混控器定义文件的设备的不同而发生改变。

当将混控器用于混合飞机的控制量时，编号为 0 的混控器组为飞机的姿态控制组，该控制组内编号 0 - 3 的选项通常分别便是滚转、俯仰、偏航和推力。

剩下的字段则是使用上文提及的缩放参数对控制量的缩放器进行了设定。 同时，结果的计算是以浮点计算的形式进行的，在混控器定义文件中的值都将扩大 10000 倍，比如：实际中 -0.5 的偏移量（offset）在定义文件中保存为 -5000 。

[这里](../airframes/adding_a_new_frame.md#mixer-file) 是一个典型混控器的示例文件。

#### 针对多旋翼的混控器

多旋翼的混控器将四组控制输入（俯仰、滚转、偏航和推力）整合到一组用于驱动电机转速控制器的执行器输出指令中。

该混控器使用如下形式的行进行定义：

    R: <geometry> <roll scale> <pitch scale> <yaw scale> <idlespeed>
    

支持的多旋翼类型为：

* 4x - X 构型的四旋翼
* 4+ - + 构型的四旋翼
* 6x - X 构型的六旋翼
* 6+ - + 构型的六旋翼
* 8x - X 构型的八旋翼
* 8+ - + 构型的八旋翼

滚转、俯仰和偏航的缩放因子大小都分别表示滚转、俯仰和边行控制相对于推力控制的比例。 同时，结果的计算是以浮点计算的形式进行的，在混控器定义文件中的值都将扩大 10000 倍，比如：实际中 0.5 的偏移量（offset）在定义文件中保存为 5000 。

滚转、俯仰和偏航输入量的范围应在 -1.0 到 1.0 之间，推力输入应该在 0.0 到 1.0 之间。每一个执行器的输出量应在 -1.0 到 1.0 之间。

怠速的设定值应在 0.0 到 1.0 之间。在这里怠速的值表示的是相对电机最大转速的百分比，当所有控制输入均为 0 的时候电机应在该转速下运行。

当有一个执行器出现饱和后，所有执行器的值都将被重新缩放以使得饱和执行器的输出被限制在 1.0 。

#### 针对直升机的混控器

直升机的混控器将三组控制输入（滚转、俯仰和推力）整合到四个输出中（倾斜盘舵机和主电机 ESC 设定）。 直升机混控器的第一个输出量是主电机的油门设定。 随后是倾斜盘舵机的指令。 尾桨的控制可以通过添加一个 简单的混控器来实现：

推力控制输入同时用于设定直升机的主电机和倾斜盘的总距。 在运行时它会使用一条油门曲线和一条总距曲线，这两条曲线都由 5 个控制点组成。

> **Note** 油门曲线及螺距曲线将 “推力” 摇杆输入位置映射到一个油门值和螺距值（单独地）。 这就使得我们可以为不同类型的飞行，对飞机的飞行特性进行调整。 如何调整这些映射曲线可以参考 [这篇指南](https://www.rchelicopterfun.com/rc-helicopter-radios.html) (搜索 *Programmable Throttle Curves* 和 *Programmable Pitch Curves*)。

混控器的定义的开头如下：

    H: <number of swash-plate servos, either 3 or 4>
    T: <throttle setting at thrust: 0%> <25%> <50%> <75%> <100%>
    P: <collective pitch at thrust: 0%> <25%> <50%> <75%> <100%>
    

`T：` 定义了油门曲线的控制点。 `P：` 定义了螺距曲线的控制点。 两条曲线都含包含0到10000之间的五个点。 对于简单的线性特性而言，这五个点的取值应该为 `0 2500 5000 7500 10000` 。

后面的各行则是对每个倾斜盘舵机 (3 个或者 4 个) 进行设定，文本行的形式如下：

    S: &lt;angle&gt; &lt;arm length&gt; &lt;scale&gt; &lt;offset&gt; &lt;lower limit&gt; &lt;upper limit&gt;
    

`<angle>` 是角度制， 0 ° 表示的倾斜盘的朝向与机头的方向相同。 从飞机上方往下看，顺时针旋转为正。 `<arm length>` 表示的是归一化的长度，文件中若值为 10000 则实际表示 1。 如果所有的舵机摇臂的长度都一致，那么这个值应该设置为 10000 。 更长的摇臂意味着舵机的偏转量更少，而较短的摇臂则意味着更多的舵机偏转量。

舵机的输出按照比例 `<scale>/10000` 进行缩放。 完成缩放后 `<offset>` 生效，该参数的取值介于 -10000 和 10000 之间。 `<lower limit>` 和 `<upper limit>` 应分别设置为 -10000 和 +10000 以使得舵机可以实现全行程。

尾桨的控制可以通过额外添加一个 [简单的混控器](#simple-mixer) 来实现：

    M: 1
    S: 0 2  10000  10000      0 -10000  10000
    

完成上述工作后，直升机的尾桨设定直接映射到了飞机的偏航指令上。 该设置同时适用于舵机控制的尾桨和使用专用电机控制的尾桨。

以 [Blade 130 直升机混控器](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/mixers/blade130.main.mix) 为例。 它的油门曲线刚开始时斜率很陡，在 50% 油门位置便达到了 6000(0.6)。 随后油门曲线会以一个稍平缓的斜率实现在 100% 油门位置时到达 10000（1.0）。 螺距曲线是线性的，但没有用到全部的控制指令区间。 0% 油门位置时总距设置就已经是 500（0.05）了。 油门处于最大位置时螺距仅仅为 4500（0.45）。 对于该型直升机而言使用更高的值会导致主桨叶失速。 该直升机的倾斜盘舵机分别位于 0°、140°、和 220° 的相位位置上。 舵机摇臂的长度并不相等。 第二个和第三个舵机的摇臂更长，其长度大约为第一个舵机的摇臂长度的 1.3054 倍。 由于机械结构限制，所有舵机均被限制在 -8000 和 8000 之间。