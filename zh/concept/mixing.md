---
translated_page: https://github.com/PX4/Devguide/blob/master/en/concept/mixing.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 混控和执行器

PX4架构保证了核心控制器中不需要针对机身布局做特别处理。

混控指的是把输入指令（例如：遥控器打`右转`）分配到电机以及舵机的执行器（如电调或舵机PWM）指令。对于固定翼的副翼控制而言，每个副翼由一个舵机控制，那么混控的意义就是控制其中一个副翼抬起而另一个副翼落下。同样的，对多旋翼而言，俯仰操作需要改变所有电机的转速。 

将混控逻辑从实际姿态控制器中分离出来可以大大提高复用性。 

## 控制流程

一个特定的控制器（如姿态控制器）发送特定的归一化（-1..+1）的命令到给混合（mixing）,然后混合后输出独立的PWM到执行器（电调，舵机等）.在经过输出驱动如（串口，UAVCAN，PWM）等将归一化的值再转回特性的值（如输出1300的PWM等）。

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIGF0dF9jdHJsW0F0dGl0dWRlIENvbnRyb2xsZXJdIC0tPiBhY3RfZ3JvdXAwW0FjdHVhdG9yIENvbnRyb2wgR3JvdXAgMF1cbiAgZ2ltYmFsX2N0cmxbR2ltYmFsIENvbnRyb2xsZXJdIC0tPiBhY3RfZ3JvdXAyW0FjdHVhdG9yIENvbnRyb2wgR3JvdXAgMl1cbiAgYWN0X2dyb3VwMCAtLT4gb3V0cHV0X2dyb3VwNVtBY3R1YXRvciA1XVxuICBhY3RfZ3JvdXAwIC0tPiBvdXRwdXRfZ3JvdXA2W0FjdHVhdG9yIDZdXG4gIGFjdF9ncm91cDJbQWN0dWF0b3IgQ29udHJvbCBHcm91cCAyXSAtLT4gb3V0cHV0X2dyb3VwMFtBY3R1YXRvciA1XSIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIGF0dF9jdHJsW0F0dGl0dWRlIENvbnRyb2xsZXJdIC0tPiBhY3RfZ3JvdXAwW0FjdHVhdG9yIENvbnRyb2wgR3JvdXAgMF1cbiAgZ2ltYmFsX2N0cmxbR2ltYmFsIENvbnRyb2xsZXJdIC0tPiBhY3RfZ3JvdXAyW0FjdHVhdG9yIENvbnRyb2wgR3JvdXAgMl1cbiAgYWN0X2dyb3VwMCAtLT4gb3V0cHV0X2dyb3VwNVtBY3R1YXRvciA1XVxuICBhY3RfZ3JvdXAwIC0tPiBvdXRwdXRfZ3JvdXA2W0FjdHVhdG9yIDZdXG4gIGFjdF9ncm91cDJbQWN0dWF0b3IgQ29udHJvbCBHcm91cCAyXSAtLT4gb3V0cHV0X2dyb3VwMFtBY3R1YXRvciA1XSIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)

## 控制组

PX4 有输入组和输出组的概念，顾名思义：控制输入组（如： `attitude`），就是用于核心的飞行姿态控制，（如： `gimbal` ）就是用于挂载控制. 一个输出组就是一个物理总线，如前8个PWM组成的总线用于舵机控制，组内带8个归一化（-1..+1）值,一个混合就是用于输入和输出连接方式（如:对于四轴来说,输入组有俯仰，翻滚，偏航等，对于于向前打俯仰操作，就需要改变输出组中的4个电调的PWM输出值，前俩个降低转速，后两个增加转速，飞机就向前）。

对于简单的固定翼来说，输入0（roll），就直接连接到输出的0（副翼）。对于多旋翼来说就不同了，输入0（roll）需要连接到所有的4个电机。

#### Control Group #0 (Flight Control)

 * 0: roll (-1..1)
 * 1: pitch (-1..1)
 * 2: yaw (-1..1)
 * 3: throttle (0..1 normal range, -1..1 for variable pitch / thrust reversers)
 * 4: flaps (-1..1)
 * 5: spoilers (-1..1)
 * 6: airbrakes (-1..1)
 * 7: landing gear (-1..1)

#### Control Group #1 (Flight Control VTOL/Alternate)

 * 0: roll ALT (-1..1)
 * 1: pitch ALT (-1..1)
 * 2: yaw ALT (-1..1)
 * 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
 * 4: reserved / aux0
 * 5: reserved / aux1
 * 6: reserved / aux2
 * 7: reserved / aux3

#### Control Group #2 (Gimbal)

 * 0: gimbal roll
 * 1: gimbal pitch
 * 2: gimbal yaw
 * 3: gimbal shutter
 * 4: reserved
 * 5: reserved
 * 6: reserved
 * 7: reserved (parachute, -1..1)

#### Control Group #3 (Manual Passthrough)

 * 0: RC roll
 * 1: RC pitch
 * 2: RC yaw
 * 3: RC throttle
 * 4: RC mode switch
 * 5: RC aux1
 * 6: RC aux2
 * 7: RC aux3

#### Control Group #6 (First Payload)

 * 0: function 0 (default: parachute)
 * 1: function 1
 * 2: function 2
 * 3: function 3
 * 4: function 4
 * 5: function 5
 * 6: function 6
 * 7: function 7

### Virtual Control Groups

These groups are NOT mixer inputs, but serve as meta-channels to feed fixed wing and multicopter controller outputs into the VTOL governor module.

#### Control Group #4 (Flight Control MC VIRTUAL)

 * 0: roll ALT (-1..1)
 * 1: pitch ALT (-1..1)
 * 2: yaw ALT (-1..1)
 * 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
 * 4: reserved / aux0
 * 5: reserved / aux1
 * 6: reserved / aux2
 * 7: reserved / aux3

#### Control Group #5 (Flight Control FW VIRTUAL)

 * 0: roll ALT (-1..1)
 * 1: pitch ALT (-1..1)
 * 2: yaw ALT (-1..1)
 * 3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
 * 4: reserved / aux0
 * 5: reserved / aux1
 * 6: reserved / aux2
 * 7: reserved / aux3

## 映射

因为存在许多控制组（例如飞行控制组，载荷组等）和许多输出组（例如基本8路PWM输出组，UAVCAN组等），所以一个控制组可以向多个输出组发送指令。

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGFjdHVhdG9yX2dyb3VwXzAtLT5vdXRwdXRfZ3JvdXBfNVxuICBhY3R1YXRvcl9ncm91cF8wLS0-b3V0cHV0X2dyb3VwXzZcbiAgYWN0dWF0b3JfZ3JvdXBfMS0tPm91dHB1dF9ncm91cF8wIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGFjdHVhdG9yX2dyb3VwXzAtLT5vdXRwdXRfZ3JvdXBfNVxuICBhY3R1YXRvcl9ncm91cF8wLS0-b3V0cHV0X2dyb3VwXzZcbiAgYWN0dWF0b3JfZ3JvdXBfMS0tPm91dHB1dF9ncm91cF8wIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)

## PX4混控器定义

`ROMFS/px4fmu_common/mixers`中的文件实现了预定义机架所使用的混控器。它们可以用于自定义机架或者一般的测试。 

### 语法

mixer通过文本文件定义；以单个大写字母加一个冒号开始的行是有效的。其它的行则会被忽略，这意味着注释可以自由地在定义中穿插使用。 

每个文件可以定义多个混控器；混控器与作动器的分配关系由读取混控器定义的设备决定，作动器输出数目则由混控器决定。

例如：每个简单混控器或者空混控器按照它们在混控器文件中出现的顺序对应到输出1到输出x。

一个混控器定义通常具有如下形式：

	<tag>: <mixer arguments>

tag标签决定混控器的类型；`M`对应简单求和混控器，`R`对应多旋翼混控器，等等。

#### 空混控器 ####

空混控器不接受控制输入并产生单个作动器输出，其输出值恒为零。空混控器的典型用法是在一组定义作动器特定输出模式的混控器组中占位。

空混控器定义形式如下：

	Z:

#### 简单混控器 ####

简单混控器将0个或多个控制输入混合为单个作动器输出。所有输入被缩放后，经过混合函数得到混合后的输入，最后再经过输出缩放产生输出信号。

简单混控器定义如下：

	M: <control count>
	O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>

如果 `<control count>` 为0，那么混合结果实际上为0，混控器将输出一个定值，这个值是在`<lower limit>`和`<upper limit>`限制下的`<offset>`。

第二行用前文讨论过的缩放参数定义了输出缩放器。计算以浮点操作被执行，存储在定义文件中的值经过了因子10000的缩放，即偏移量-0.5会被存储为-5000。

紧跟在`<control count>`词目之后的定义描述了控制输入以及它们的缩放，形式如下：

	S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
`<group>`值标示了控制输入来源，缩放器从中读取控制量，`<index>`值则是控制量在组内的序号。这些值对读取混控器定义的设备而言都是特定的。

当用来混合载体控制时，控制组0是载体姿态控制组，序号0到3通常对应滚转，俯仰，偏航和油门。

混控器定义行中剩下的域则用来配置缩放器，参数如前文讨论。计算以浮点操作被执行，存储在定义文件中的值经过了因子10000的缩放，即偏移量-0.5会被存储为-5000。

#### 多旋翼混控器 ####

多旋翼混控器将4个控制输入（滚转，俯仰，偏航，油门）混合至一组作动器输出，这些作动器用来驱动电机转速控制器。

多旋翼混控器定义如下所示：

	R: <geometry> <roll scale> <pitch scale> <yaw scale> <deadband>

支持的构型包括：

 * 4x - X型布局四旋翼
* 4+ - +型布局四旋翼
* 6x - X型布局六旋翼
* 6+ - +型布局六旋翼
* 8x - X型布局八旋翼
* 8+ - +型布局八旋翼

每个滚转，俯仰，偏航缩放值定义了滚转，俯仰，偏航控制相对于油门控制的缩放。计算以浮点操作被执行，存储在定义文件中的值经过了因子10000的缩放，即偏移量-0.5会被存储为-5000。

滚转，俯仰和偏航输入的范围为-1.0到1.0，而油门输入的范围为0.0到1.0，执行器输出范围为-1.0到1.0。

当某个执行器饱和时，为保证该执行器值不超出范围，所有的执行器值都会被重新缩放，使得执行器的饱和上限被限制到1.0以内。