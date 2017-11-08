# Hardware in the Loop Simulation (HITL)

HITL은 autopilot이 시뮬레이터에 연결하고 모든 flight mode가 autopilot에서 실행되는 시뮬레이션 모드입니다. 이 방식은 실제 프로세서에서 실제 비행 코드를 테스팅하는 장점을 가지고 있습니다.

## HITL에 대한 시스템 설정

PX4는 멀티콥터(jMAVSim을 사용)와 고정익을(X-Plane demo나 모두 사용) 위해 HITL을 지원합니다. Flightgear 지원도 잘 되고 있지만 X-Plane를 추천합니다. 이를 활성화시키려면 airframe 메뉴를 통해 설정합니다.

![QGroundControl HITL configuration](../../assets/gcs/qgc_hil_config.png)

## jMAVSim (쿼드로터) 사용하기 {#using-jmavsim-quadrotor}

- QGroundControl가 실행 중이 아님을 확인(혹은 시리얼 포트로 장치 연결 중)
- HITL 모드엥서 jMAVSim을 실행 (필요하면 serial 포트를 교체):
  ```
  ./Tools/jmavsim_run.sh -q -d /dev/ttyACM0 -b 921600 -r 250
  ```
- 콘솔은 autopilot으로부터의 mavlink 텍스트 메시지를 출력합니다.
- 다음으로 QGroundControl를 실행하고 UDP 디폴트 설정으로 연결합니다.

## X-Plane 사용하기
#### X-Plane에 리모트 접근을 활성화

X-Plane에서 해야할 2가지 핵심 셋팅 :  Settings -> Data Input and Output 에서 체크박스 설정:

![](../../assets/gcs/xplane_data_config.png)

Data 탭에서 Settings -> Net Connections에서 localhost와 port 49005를 IP 주소로 설정. 아래 스크린샷 참조:

![](../../assets/gcs/xplane_net_config.png)

#### QGroundControl에서 HITL 활성화

Widgets -> HIL Config, 다음으로 드롭다운에서 X-Plane 10을 선택하고 연결. 일단 시스템이 연결되면 배터리 상태, GPS 상태, 비행기 위치가 활성화됩니다 :

![](../../assets/gcs/qgc_sim_run.png)

## Joystick 입력으로 스위치

라디오 리모트 컨트롤보다 조이스틱을 선호한다면, `COM_RC_IN_MODE`을 `1`로 설정합니다. Commander 파라미터 그룹에서 찾을 수 있습니다.

## HITL에서 자동 mission 비행

비행 planning view로 전환해서 비행체 앞에 한개 waypoint를 찍습니다. waypoint를 전송하기 위해서 sync 아이콘을 클릭합니다.

다음으로 툴바에 있는 비행 모드 메뉴에서 MISSION을 선택하고 비행기를 arm하기 위해서 DISARMED를 클릭합니다. 이륙해서 waypoint 주변에 떠있게 됩니다.

![](../../assets/gcs/qgc_sim_mission.png)
