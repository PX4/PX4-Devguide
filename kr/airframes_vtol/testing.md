# VTOL 테스팅

VTOL 기능을 제대로 동작하는지에 대한 기본 테스트는 주로 transition에 초점 :

  * 대기 상태 (On the bench)
  * 비행 중 (In flight)

## transition 중 일반 주의내용

현재 제공하는 VTOL에 transiton 명령을 내리는 3가지 방식 :

  * RC 스위치 (2 pos, aux1)
  * MAVLink 명령 (MAV_CMD_DO_VTOL_TRANSITION)
  * mission 중에 transition (내부적으로 MAV_CMD_DO_VTOL_TRANSITION)

위 방법중에 하나로 transition 명령이 내리면, VTOL은 transition phase로 들어갑니다. 만약 VTOL이 transition을 진행 중인 상태에서 이전 상태로 돌아가는 새로운 transition 명령을 받으면, 바로 이전 상태로 돌아갑니다. 필요한 경우 transition을 취소하는 것은 안전과 관련된 기능입니다. transition이 완료되면, VTOL은 새로운 상태로 들어갔으므로 반대 방향의 transition 명령은 정상적으로 수행하게 됩니다.

> **Note**
AUX1 채널을 RC 스위치에 할당하고 airspeed가 정상적으로 동작하는지 확인합니다.

## 대기 상태 (On the bench)

> **Caution** 모든 프로펠러를 제거합니다! transition 기능을 제대로 테스트하기 위해서 비행체는 arm 상태로 되어 있어야 합니다.

기본적으로 멀티로터 모드에서 시작 :

  * 비행체 arm
  * 모터가 멀티로터 설정에서 실행되는지 확인 (rudders/elevons은 반드시 roll/pitch/yaw 입력에도 움직이면 안됨)
  * transition 스위치를 토글
  * (가능하면) transition의 1단계가 완료될때까지 대기
  * airspeed를 시뮬레이션하기 위해서 pito에 바람 불어보
  * (가능하면) transition의 2단계가 실행
  * 고정익 설절에서 모터가 실행되는지 확인 (roll/pitch/yaw 입력으로 rudders/elevons를 제어할 수 있어야함)
  * transition 스위치를 토글
  * 이전 상태로 돌아가는 것을 관찰
  * 모터가 멀티로터 설정에서 실행되는지 확인 (rudders/elevons는 roll/pitch/yaw입력에도 움직이면 안됨)

## 비행 중 (In flight)

> **Tip** 비행 중에 transition을 테스트하기 전에, VTOL이 멀티로터 모드에서 안정적으로 비행하는지 확인합니다. 계획한대로 제대로 동작하지 않는 경우, 멀티로터 모드로 전환하고 복구되도록 합니다.(제대로 튜닝이 되고나야 제대로 동작)

비행 중에 transition을 하기 위해서는 여러분이 가지고 있는 비행체와 비행조정 기술에 맞게 최소한 다음 parameter가 필요합니다. :

| Param | Notes |
| :--- | :--- |
| VT_FW_PERM_STAB | 고정익에 대해서 영구 안정화 on/off 시키기 |
| VT_ARSP_BLEND | 특정 airspeed에서 고정익 제어를 활성화 |
| VT_ARSP_TRANS | 특정 airspeed에서 고정익 전환을 완료 |

VTOL 타입에 따라서 더 다양한 parameter이 있습니다. [parameter reference](https://pixhawk.org/firmware/parameters#vtol_attitude_control)를 참고하세요.

### 수동 transition 테스트

수동 transition을 테스트하기 위한 기본 절차는 다음과 같다 :

  * 멀티로터 모드에서 arm과 이륙
  * transition 이후에 조금 아래로 내려올 수 있으므로 안전한 높이까지 올라가기
  * 바람방향으로 돌리기
  * transition 스위치를 토글
  * transition 관찰 **(MC-FW)**
  * 고정익으로 비행
  * transition 이후에 조금 아래로 내려올 수 있으므로 안전한 높이로 이동
  * transition 스위치를 토글
  * transition 관찰 **(FW-MC)**
  * 착륙 및 disarm

**MC-FW**

MC에서 FW로 transition하는 동안 다음과 같은 일이 일어납니다 :

  1. 속도가 높으면서 제어가 되지 않는다 (여러 가지 요인이 있음)
  2. transition에 너무 오래 걸리고 transition이 끝나기 전에 너무 멀리 날아가 버린다.

1)에 대해서 : 스위치를 멀티로터로 되돌린다. (바로) 무슨 문제인지를 찾는다.(setpoint 확인하기)

2)에 대해서 : blending airspeed가 설정되어 있고 이미 더 높은 airspeed인 경우라면 고정익으로 제어가능합니다. 따라서 주변을 비행하면서 transition는 완료하는데 필요한 시간을 더 주어야 합니다. 그렇지 않은 경우 멀티로터로 스위치를 되돌리고 무슨 문제인지를 찾습니다.(airspeed 확인)

**FW-MC**

FW에서 MC로 transition하는 경우 거의 문제가 발생하지 않습니다. 만약 제어가 되지 않는 경우 가장 좋은 방법은 복귀시키는 것입니다.

### 자동 transition 테스트 (mission, commanded)

명령으로 transition을 받는 경우는 auto(mission)이나 offboard 비행모드에서만 동작합니다. 비행중에 auto/offboard와 transition 스위치가 제대로 동작하는지를 확신할 수 있어야 합니다.

수동으로 스위칭하면 transition 스위치를 재활성화시켜야 합니다. 예를 들자면 auto/offboard 이외 모드로 되어 있고 자동으로 고정익 비행에 transition 스위치가 현재 멀티로터로 되어 있다면 바로 멀티로터로 전환될 것입니다.

#### 절차

transtion이 있는 mission을 테스트하는데 사용하는 절차는 :

  * mission을 업로드
  * 멀티로터 모드로 이륙하고 mission 높이까지 올라가기
  * 스위치로 mission 활성화  
  * 고정익 비행으로 전환되는 것 관찰하기
  * 비행 즐기기
  * 다시 멀티로터 모드로 전환되는 것 관찰하기
  * mission 비활성화
  * 수동으로 착륙

비행 동안에 수동으로 transition 스위치를 멀티로터로 유지합니다. 잘 되지 않으면, 스위치를 수동하면 멀티로터 모드로 복귀됩니다.

#### 예제 mission

mission에서 최소한 포함해야 하는 것들(아래 스크린샷 참조) :

  * (1) 이륙 장소 근처에 position waypoint
  * (2) 사전 계획한 고정익 비행 경로의 방향으로 position waypoint
  * (3) transition waypoint (plane mode로)
  * (4) 더 떨어진 position waypoint (적어도 transition에 필요한 거리 만큼은 떨어져 있어야)
  * (6) 다시 되돌아 가기 위한 position waypoint (이륙 위치 약간 앞쪽. 다시 전환에 필요한 거리)
  * (7) transition waypoint (hover mode로)
  * (8) 이륙 위치 근처에 position waypoint

![Mission, showing transition WP to plane](../../assets/vtol/qgc_mission_example_a.png)

![Mission, showing transition WP to hover](../../assets/vtol/qgc_mission_example_b.png)
