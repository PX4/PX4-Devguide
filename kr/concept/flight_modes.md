# Flight Modes

**Flight Modes** 는 언제라도 시스템의 상태를 정의할 수 있습니다. flight mode 사이에 사용자 전환은 리모트 콘트롤의 스위치를 이용하는 방법과 [ground control station](../qgc/README.md)을 이용하는 방법이 있습니다.

모든 flight mode가 모든 비행체 타입에서는 사용할 수 있는 것은 아닙니다. 일부 모드는 비행체 타입에 따라서 다른 동작을 하기도 합니다.(아래 설명 참고) 마지막으로 일부 flight mode들은 특정 사전 비행과 비행 중 상태에서만 동작합니다.(예로 GPS lock, airspeed 센서, 축을 따라 비행체 고도 센싱) 해상 시스템은 적절한 상태 조건이 아닌 경우에는 다른 모드로 전환이 되지 않습니다.

아래 섹션에서 여러 모드에 대한 개요와 PX4가 새로운 모드로 전환되는 조건을 보여주는 [flight mode evaluation 다이어그램](#flight-mode-evaluation-diagram)에 소개합니다.


## Flight Mode 간략 요약

### Manual 비행 모드

"Manual" 모드는 사용자가 RC 컨트롤(혹은 조이스틱)을 통해서 직접 비행체를 제어합니다. 비행체 움직임은 스틱의 움직임을 따릅니다. 하지만 응답의 레벨/타입은 해당 모드에 따라 다릅니다. 예로 경험이 많은 사용자는 액츄레이터로 스틱의 조정을 직접 전달하는 모드를 사용할 수 있습니다. 반면에 초급자는 갑작스런 스틱 변경에 덜 민감하게 동작하는 모드를 선택하는게 좋습니다.

* **Fixed wing aircraft/ rovers / boats:**
  * **MANUAL:** 조정자의 제어 입력이 그대로 출력 mixer로 전달
  * **STABILIZED:** 조정자의 입력 중 roll과 pitch는 *angle* command로 yaw는 manual 명령.
* **Multirotors:**
  * **ACRO:** 조정자의 입력은 roll, pitch, yaw *rate* command로 비행체에 전달. multirotor를 완전히 뒤집기가 가능. throttle은 직접 출력 mixer로 전달.
  * **RATTITUDE** 조정자의 입력은 해당 mode의 임계값보다 큰 경우 roll, pitch, yaw *rate* command로 비행체에 전달. 만약 임계값보다 크지 않다면 roll과 pitch는 *angle* command로 yaw는 *rate* command로 전달. throttle은 직접 출력 mixer로 전달.
  * **STABILIZED** 조정자의 입력은 roll과 pitch *angle* command로 전달되고 yaw는 *rate* command로 전달. throttle은 직접 output mixer로 전달.

### Assisted 비행 모드

"Assisted" 모드도 사용자가 제어하는 모드지만 일부는 "자동화" 도움을 받습니다. - 예로 바람이 불더라도 자동으로 위치/방향을 유지합니다. Assisted 모드는 비행 제어를 쉽도록 도와줍니다.

* **ALTCTL** (Altitude Control)
  * **Fixed wing aircraft:** roll, pitch 그리고 yaw 입력이 모두 가운데 위치할때(지정한 deadband 범위을 넘지 않는 경우), 비행체는 현재 고도를 유지하며 똑바로 돌아온다. 바람이 부는 경우 drift가 발생할 수 있다.
  * **Multirotors:** roll, picth 그리고 yaw 입력은 MANUAL mode와 동일. throttle 입력은 사전에 지정한 최대 rate에서 위나 아래로 이동 지시. throttle은 넓은 deadzone을 가짐.
* **POSCTL** (Position Control)
  * **Fixed wing aircraft:** 중립 입력일 때 수평으로 비행하며 직선으로 유지해야 하는 경우라면 바람때문에 문제될 수 있음.
  * **Multirotors** roll은 왼쪽-오른쪽 속도를 제어하고 pitch는 앞-뒤 속도를 제어한다. roll과 pitch가 모두 가운데(deadzone 내부) 있는 경우, 멀티로터는 position을 유지하게 됩니다. yaw는 MANUAL mode와 같이 yaw rate를 제어. throttle은 상승/하강 rate를 ALTCTL mode처럼 제어.

### Auto 비행 모드

"Auto" 모드는 사용자의 입력이 거의 없이 제어가 가능한 모드입니다. (예로 이륙/착륙과 fly mission들)

* **AUTO_LOITER** (Loiter)
  * **Fixed wing aircraft:** 비행체가 현재 고도와 현재 위치 주위를 비행함.(혹은 현재 고도보다 약간 높을 수도 있음)
  * **Multirotors:** 멀티로터는 현재 위치와 고도에서 비행.
* **AUTO_RTL** (Return to Land)
  * **Fixed wing aircraft:** 비행체는 홈 위치로 돌아와서 홈 주위를 원을 그리면서 비행.
  * **Multirotors:** 멀티로터는 현재 고도에서 직선(home position + loiter altitude보다 높은 경우) 혹은 loiter altitude(현대 고도보다 높은 경우)로 돌아온다. 다음으로 자동 착륙.
* **AUTO_MISSION** (Mission)
  * **All system types:** 비행체는 GCS가 보낸 프로그램된 mission을 따름. 만약 수신한 mission이 없는 경우에는 비행체는 현재 위치에서 LOITER하게 됨.
  * **_OFFBOARD_** (Offboard)
    이 모드에서 position, velocity 그리고 attitude reference / target / setpoint은 시리얼 케이블과 MAVLink로 연결된 컴패니온 컴퓨터에서 제공합니다. offboard setpoint는 [MAVROS](https://github.com/mavlink/mavros) 나 [Dronekit](http://dronekit.io)와 같은 API를 제공할 수 있습니다.

## Flight Mode Evaluation 다이어그램 {#flight-mode-evaluation-diagram}

![Flight Mode Evaluation Diagram](../../assets/diagrams/commander-flow-diagram.png)
