# 飞行模式

*飞行模式* 定义自动驾驶仪如何响应用户输入并控制飞机移动。 可以根据自动驾驶仪介入的程度将飞行模式粗略地分为 *manual*, *assisted* 和 *auto* 三大模式。 飞行员使用遥控器上的开关或者 [ground control station](../qgc/README.md) 在飞行模式之间进行切换。

需要注意的是并非所有类型的飞机都具备全部的飞行模式，同时部分模式在不同类型的飞机上的行为模式也不相同（见下文）。 最后，部分飞行模式仅在飞行前或者飞行中某些特定条件下才有意义（如 GPS锁定，空速传感器，飞机扰一个轴进行姿态感知）。 除非满足合适的条件，都则系统不会允许切换到这些模式下。

下面的各小节对所以的飞行模式进行了一个概述，随后给出了一张 [飞行模式评价图](#flight-mode-evaluation-diagram) ，改图展示了 PX4 在何种条件下会切换至一个新的飞行模式。

## 飞行模式概要

### 手动飞行模式

“手动”飞行模式下用户可通过 RC 遥控器（或操纵杆）实现对飞机的直接控制。 飞机的运动总是跟锁着摇杆的运行，但响应的级别/类型会根据模式的不同而发生变化。 例如，有经验的飞手可以使用直接将摇杆位置传递给执行器的模式，而新手则通常选择一些对杆位突变反应不是很灵敏的飞行模式。

* **固定翼飞机/无人车/无人船：**
  
  * **MANUAL：** 飞行员的控制输入（来自 RC 控制器的原始用户输入）直接传递给输出混控器
  * **STABILIZED：** 飞行员的俯仰和滚转输入将作为角度指令传递给自动驾驶仪，而偏航输入则由输出混控器直接传递给方向舵（手动控制）。 如果 RC 遥控器的滚转和俯仰摇杆处于居中位置，那么自动驾驶仪会将飞机的滚转角和俯仰角调整为零，因此可以在风的扰动下稳定飞机姿态（平飞）。 但是。在此模式下飞机的位置不受自驾仪的控制，因此飞机的位置可能会由于风的存在而发生漂移。 非零滚转角输入的情况下飞机会进行协调转弯以实现无侧滑（y方向的（侧向）加速度为零）。 在协调转弯时，方向舵用于控制侧滑角，任何手动的偏航输入都会叠加在该舵面上。
  * **ACRO：** 飞行员的输入将作为滚转、俯仰和偏航 *角速率* 指令传递给自动驾驶仪 自动驾驶仪控制非机动角速度。 油门将直接传递到输出混控器上。

* **多旋翼：**
  
  * **MANUAL/STABILIZED：** 飞行员的输入将作为滚转、俯仰 *角度* 指令和一个偏航 *角速度* 指令传递给自动驾驶仪， 油门将直接传递到输出混控器上。 The autopilot controls the attitude, meaning it regulates the roll and pitch angles to zero when the RC sticks are centered, consequently leveling-out the attitude. However, in this mode the position of the vehicle is not controlled by the autopilot, hence the position can drift due to wind.
    
    > **Note** For Multirotors, Manual and Stabilized modes are the same.
  
  * **ACRO:** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot. The autopilot controls the angular rates, but not the attitude. Hence, if the RC sticks are centered the vehicle will not level-out. This allows the multirotor to become completely inverted. 油门直接传递到输出混频器。
  
  * **RATTITUDE** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot if they are greater than the mode's threshold, i.e. if the RC sticks are a certain distance away from the center position. If not the inputs are passed as roll and pitch *angle* commands and a yaw *rate* command. 油门直接传递到输出混频器。 In short, the autopilot acts as an angular rate controller when the RC sticks are away from center (like in the ACRO mode), whereas when the RC sticks are centered, the autopilot acts as an attitude controller (like in the Stabilized mode).

### Assisted flight modes

"Assisted" modes are also user controlled but offer some level of "automatic" assistance - for example, automatically holding position/direction, against wind. Assisted modes often make it much easier to gain or restore controlled flight.

* **ALTCTL** (Altitude Control) 
  * **Fixed wing aircraft:** When the roll, pitch and yaw (RPY) RC sticks are all centered (or less than some specified deadband range) the aircraft will return to straight and level flight and keep its current altitude. Its x and y position will drift with the wind.
  * **Multirotors:** Roll, pitch and yaw inputs are as in Stabilised mode. Throttle inputs indicate climb or sink at a predetermined maximum rate. Throttle has large deadzone. Centered Throttle holds altitude steady. The autopilot only controls altitude so the x,y position of the vehicle can drift due to wind. 
* **POSCTL** (Position Control) 
  * **Fixed wing aircraft:** Neutral inputs (centered RC sticks) give level flight and it will crab against the wind if needed to maintain a straight line.
  * **Multirotors** Roll controls left-right speed, pitch controls front-back speed over ground. Yaw controls yaw rate as in MANUAL mode. Throttle controls climb/descent rate as in ALTCTL mode. This means that the x, y, z position of the vehicle is held steady by the autopilot against any wind disturbances, when the roll, pitch and throttle sticks are centered.

### Auto flight modes

"Auto" modes are those where the controller requires little to no user input (e.g. to takeoff, land and fly missions).

* **AUTO_LOITER** (Loiter) 
  * **Fixed wing aircraft:** The aircraft loiters around the current position at the current altitude (or possibly slightly above the current altitude, good for 'I'm losing it'). 
  * **Multirotors:** The multirotor hovers / loiters at the current position and altitude.
* **AUTO_RTL** (Return to Land) 
  * **Fixed wing aircraft:** The aircraft returns to the home position and loiters in a circle above the home position. 
  * **Multirotors:** The multirotor returns in a straight line on the current altitude (if the current altitude is higher than the home position + [RTL_RETURN_ALT](../advanced/parameter_reference.md#RTL_RETURN_ALT)) or on the [RTL_RETURN_ALT](../advanced/parameter_reference.md#RTL_RETURN_ALT) (if the [RTL_RETURN_ALT](../advanced/parameter_reference.md#RTL_RETURN_ALT) is higher than the current altitude), then lands automatically.
* **AUTO_MISSION** (Mission) 
  * **All system types:** The aircraft obeys the programmed mission sent by the ground control station (GCS). If no mission received, aircraft will LOITER at current position instead.
  * ***OFFBOARD*** (Offboard) In this mode the position, velocity or attitude reference / target / setpoint is provided by a companion computer connected via serial cable and MAVLink. The offboard setpoint can be provided by APIs like [MAVROS](https://github.com/mavlink/mavros) or [Dronekit](http://dronekit.io).

## Flight Mode Evaluation Diagram

![](../../assets/diagrams/commander-flow-diagram.png)