# Flight Modes

*Flight Modes*는 사용자의 입력에 대해 어떻게 반응하고 기체를 움직일지에 대해 정의 합니다. 자동조종장치에 의해 제공되는 레벨/타입에 기초하여 *manual*, *assisted*, *auto* 모드로 나뉘어 집니다. 조종사는 원격 컨트롤러의 스위치나 [ground control station](../qgc/README.md)을 이용하여 비행 모드를 전환합니다.

모든 종류의 기체에서 모든 비행 모드가 지원되는 것은 아니며, 어떤 모드는 기체의 종류에 따라 다르게 동작합니다 (밑에 설명한 것처럼). 그리고, 어떤 비행 모드는 특정 비행 전 및 비행 중인 상황에서만 (예. GPS lock, 풍력 센서, 축을 따라 기체의 자세를 센싱) 의미가 있습니다. 이 시스템은 올바른 조건을 만족하지 않는한 다른 모드로의 전환을 허용하지 않습니다.

아래의 섹션은 모드들의 개요를 보여줍니다. [flight mode evaluation diagram](#flight-mode-evaluation-diagram)는 PX4가 새로운 모드로 전환하기 위한 조건들을 보여줍니다.

> **Note** 더 자세한 사용자용 비행 모드 문서는 [PX4 User Guide](https://docs.px4.io/master/en/flight_modes/)에서 찾을 수 있습니다.

## Flight Mode Summary

### Manual flight modes

"Manual"는 사용자가 RC 컨트롤 (또는 조이스틱) 을 통하여 기체를 직접 조종하는 모드입니다. 기체의 움직임은 항상 스틱의 움직임을 따르지만, 모드에 따라 반응의 레벨/타입은 변합니다. 예를 들어, 숙력된 비행사는 스틱위 위치를 엑츄에이터에 직접전달하는 모드를 사용할 수 있지만, 초보자는 스틱위치의 갑작스런 변환에 적게 반응하는 모드를 선택할 것입니다.

* **고정익 / 탐사선 / 보트**
  
  * **MANUAL:** 조종사의 컨트롤이 (리모트컨트롤러로부터 오는 가공되지 않은 사용자의 입력) 출력 믹서에 직접적으로 전달됩니다.
  * **STABILIZED:** 조종사의 pitch, roll 입력이 angle 커맨드로 자종조종시스템에 전달됩니다. 반면에 yaw 입력은 출력 믹서를 통해 방향타에 직접적으로 전달됩니다 (수동 조종). 만약 리모트컨트롤러의 roll, pitch 스틱이 중심을 향하면, 자동조종장치는 roll과 pitch의 각을 0으로 조절합니다. 따라서 어떠한 바람의 방해에서 기체를 안정되게 합니다. 그러나 이 모드에서는 비행기의 위치가 자동조종장치에 의해서 제어되지는 않습니다. 따라서 위치는 바람때문에 이동할 될 수 있습니다. 0이 아닌 roll 입력은 기체가 옆으로 미끄러지지 않도록 균형선회 합니다. 균형선회 하는 동안, 방향타는 옆으로 미끄러지는 것을 방지하고 어떤 yaw 입력이 더해집니다.
  * **ACRO:** 조종사의 입력이 roll, pitch, yaw *rate* 명령어로 자동조종장치에 전달됩니다. 자동조종장치는 angular rate를 제어합니다. Throttle은 출력 믹서에 직접적으로 전달됩니다.

* **멀티콥터:**
  
  * **MANUAL/STABILIZED** 조종사의 입력은 roll, pitch *angle* 명령어와 yaw *rate* 명령어로 전달됩니다. Throttle은 출력 믹서에 직접적으로 전달됩니다. 자동조종장치는 자세를 제어합니다. 즉, RC 스틱이 중심으로 모일때 roll 과 pitch angle을 0으로 조절합니다. 따라서 수평을 유지합니다. 그러나, 이 모드에서는 자동조종장치가 기체의 포지션을 제어하지는 않습니다. 따라서 포지션은 바람에 의해 바뀔 수 있습니다.
    
    > **Note** 멀티콥터는 Manual 모드와 Stabilized 모드가 동일합니다.
  
  * **ACRO:** 조종사의 입력은 roll, pitch, yaw *rate* 명령어로 자동조종장치에 전달됩니다. 자종조종장치의 angular rate는 제어하지만 자세는 제어하지 않습니다. 따라서, RC 스틱이 중심으로 모여지면 수평이 되지 않을 것입니다. 이것은 멀티콥터가 완전히 뒤집히는 것을 허용합니다. Throttle은 출력 믹서에 직접적으로 전달됩니다.
  
  * **RATTITUDE** 조종사의 입력이 threshold보다 높으면 roll, pitch, yaw *rate* 명령어로 자동조종장치에 전달됩니다. 예를 들어 RC 스틱 중심에서 확실히 멀어질 때 입니다. 그렇지 않으면 명령어가 전달되지 않습니다. Throttle은 출력 믹서에 직접적으로 전달됩니다. 다시 말해, 자동조종장치는 RC 스틱이 많이 움직이면 (ACRO모드 처럼) anulara rate 컨트롤러 역할을 하지만 그렇지 않으면 자동조종장치는 자세 컨트롤러 역할을 합니다 (Stabilized모드 처럼)

### 비행 보조 모드

보조 모드 또한 사용자가 조종하는 것이지만 몇몇의 자동화된 보조를 제공합니다. 예를 들어, 바람에 맞서 기체를 자동적으로 고정하는 것입니다. 보조 모드는 비행 제어를 보다 쉽게 만들어 줍니다.

* **ALTCTL** (고도 제어) 
  * **고정익:** When the roll, pitch and yaw (RPY) RC sticks are all centered (or less than some specified deadband range) the aircraft will return to straight and level flight and keep its current altitude. x와 y의 위치는 바람에 의해 이동될 것입니다.
  * **멀티콥터:** roll, pitch, yaw 입력은 stabilized 모드에 있게 됩니다. Throttle 입력은 미리 정의된 최대 속도에서 오르거나 내리는 신호를 보냅니다. Throttle은 큰 데드존을 갖게 됩니다. 가운데의 throttle은 고도를 안정되게 유지합니다. 자동조종장치는 단지 고도만을 조종합니다. 따라서 x,y는 바람에 의해 이동될 수 있습니다.
* **POSCTL** (위치 제어) 
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