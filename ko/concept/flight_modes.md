# Flight Modes

*Flight Modes*는 사용자의 입력에 대해 어떻게 반응하고 기체를 움직일지에 대해 정의 합니다. 자동조종장치에 의해 제공되는 레벨/타입에 기초하여 *manual*, *assisted*, *auto* 모드로 나뉘어 집니다. 조종사는 원격 컨트롤러의 스위치나 [ground control station](../qgc/README.md)을 이용하여 비행 모드를 전환합니다.

모든 종류의 기체에서 모든 비행 모드가 지원되는 것은 아니며, 어떤 모드는 기체의 종류에 따라 다르게 동작합니다 (밑에 설명한 것처럼). 그리고, 어떤 비행 모드는 특정 비행 전 및 비행 중인 상황에서만 (예. GPS lock, 풍력 센서, 축을 따라 기체의 자세를 센싱) 의미가 있습니다. 이 시스템은 올바른 조건을 만족하지 않는한 다른 모드로의 전환을 허용하지 않습니다.

아래의 섹션은 모드들의 개요를 보여줍니다. [flight mode evaluation diagram](#flight-mode-evaluation-diagram)는 PX4가 새로운 모드로 전환하기 위한 조건들을 보여줍니다.

> **Note** 더 자세한 사용자용 비행 모드 문서는 [PX4 User Guide](https://docs.px4.io/master/en/flight_modes/)에서 찾을 수 있습니다.

## Flight Mode Summary

### Manual flight modes

"Manual"는 사용자가 RC 컨트롤 (또는 조이스틱) 을 통하여 기체를 직접 조종하는 모드입니다. 기체의 움직임은 항상 스틱의 움직임을 따르지만, 모드에 따라 반응의 레벨/타입은 변합니다. For example, experienced fliers can use modes that provide direct passthrough of stick positions to actuators, while beginners will often choose modes that are less responsive to sudden stick-position changes.

* **Fixed wing aircraft/ rovers / boats:**
  
  * **MANUAL:** The pilot's control inputs (raw user inputs from RC transmitter) are passed directly to the output mixer.
  * **STABILIZED:** The pilot's pitch and roll inputs are passed as angle commands to the autopilot, while the yaw input is sent directly via the output mixer to the rudder (manual control). If the RC roll and pitch sticks are centered, the autopilot regulates the roll and pitch angles to zero, hence stabilizing (leveling-out) the attitude against any wind disturbances. However, in this mode the position of the aircraft is not controlled by the autopilot, hence the position can drift due to wind. With nonzero roll input the vehicle does a coordinated turn to achieve zero sideslip (the acceleration in y-direction (sidewards) is zero). During a coordinated turn, the rudder is used to control the sideslip and any manual yaw input is added to that.
  * **ACRO:** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot. The autopilot controls the angular rates. Throttle is passed directly to the output mixer.

* **Multirotors:**
  
  * **MANUAL/STABILIZED** The pilot's inputs are passed as roll and pitch *angle* commands and a yaw *rate* command. Throttle is passed directly to the output mixer. The autopilot controls the attitude, meaning it regulates the roll and pitch angles to zero when the RC sticks are centered, consequently leveling-out the attitude. However, in this mode the position of the vehicle is not controlled by the autopilot, hence the position can drift due to wind.
    
    > **Note** For Multirotors, Manual and Stabilized modes are the same.
  
  * **ACRO:** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot. The autopilot controls the angular rates, but not the attitude. Hence, if the RC sticks are centered the vehicle will not level-out. This allows the multirotor to become completely inverted. Throttle is passed directly to the output mixer.
  
  * **RATTITUDE** The pilot's inputs are passed as roll, pitch, and yaw *rate* commands to the autopilot if they are greater than the mode's threshold, i.e. if the RC sticks are a certain distance away from the center position. If not the inputs are passed as roll and pitch *angle* commands and a yaw *rate* command. Throttle is passed directly to the output mixer. In short, the autopilot acts as an angular rate controller when the RC sticks are away from center (like in the ACRO mode), whereas when the RC sticks are centered, the autopilot acts as an attitude controller (like in the Stabilized mode).

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
  * ***OFFBOARD*** (Offboard) In this mode the position, velocity or attitude reference / target / setpoint is provided by a companion computer connected via serial cable and MAVLink. The offboard setpoint can be provided by APIs like [MAVSDK](http://mavsdk.mavlink.io) or [MAVROS](https://github.com/mavlink/mavros).

## Flight Mode Evaluation Diagram

![Commander Flow diagram.](../../assets/diagrams/commander-flow-diagram.png)