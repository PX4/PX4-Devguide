# 비젼 혹은 모션 캡쳐 시스템 사용하기

> **Info** 아래 내용을 따라하기 전에, 여러분의 autopilot의 펌웨어 버전이 LPE 모듈이 활성화되어 있는지 확인합니다. PX4 펌웨어의 LPE 버전은 최신 PX4 릴리즈의 zip 파일 내부 혹은 `make px4fmu-v2_lpe`와 같은 빌드명령으로 소스를 빌드하면 볼수 있습니다. 상세한 내용은 [코드 빌드하기](../setup/building_px4.md)를 참고하세요.

이 페이지의 목적은 GPS이 아닌 소스로부터(VICON과 Optitrack 같은 모션 캡쳐 시스템 그리고 [ROVIO](https://github.com/ethz-asl/rovio), [SVO](https://github.com/uzh-rpg/rpg_svo) 혹은 [PTAM](https://github.com/ethz-asl/ethzasl_ptam) )와 같은 비젼 기반 estimation 시스템) 위치 데이터를 사용하는 PX4 기반 시스템을 구성하는 것입니다.

position estimate는 오프보드(예로 VICON) 뿐만 아니라 온보드 컴퓨터가 보낼 수도 있습니다. 이 데이터는 로컬 기준에 상대적인 로컬 위치 estimate을 업데이트하는데 사용합니다. 비젼/모션 캡쳐 시스템에서 헤딩은 선택적으로 attitude estimator가 통합할 수도 있습니다.

시스템은 실내 position hold나 비젼 기반 waypoint 네비게이션과 같은 어플리케이션으로 사용될 수 있습니다.

비전에 관해서 pose data를 보내는데 사용하는 mavlink 메시지는 [VISION_POSITION_ESTIMATE](http://mavlink.org/messages/common#VISION_POSITION_ESTIMATE)와 모션 캡쳐 시스템에 관한 모든 메시지는 [ATT_POS_MOCAP](http://mavlink.org/messages/common#ATT_POS_MOCAP) 메시지입니다.

mavros ROS-Mavlink 인터페이스는 기본적으로 이런 메시지를 보내는 구현을 가지고 있습니다. 이 메시지는 순수 C/C++ 코드와 MAVLink 라이브러리를 직접 사용하는 방식으로 전달할 수 있습니다.

**이 기능은 LPE estimator로만 테스트되었습니다.**

## 비젼 혹은 Mocap을 위한 LPE 튜닝

### 외부 pose 입력 활성화
시스템에서 비젼/mocap 사용을 활성인 비활성화시키기 위해서 몇가지 파라미터를 설정해야 합니다.(QGroundControl 혹은 NSH 쉘 사용)

외부 헤딩 통합 기능을 활성화시키기 위해 `ATT_EXT_HDG_M`를 1이나 2로 설정합니다. 1로 설정하면 비젼이 사용되고 2로 설정하면 mocap 헤딩을 사용합니다.

비젼 통합은 기본적으로 LPE에서 활성화됩니다. QGroundControl에서 `LPE_FUSION` 파라미터를 사용해서 제어할 수 있습니다. "fuse vision position"이 체크되어 있는지 확인합니다.

#### barometer fusion 비활성화
비젼이나 mocap 정보로부터 높은 정확도의 고도 정보를 얻을 수 있다면 Z축에 드리프트를 줄이기 이해서 LPE에서 baro 보정을 비활성화시키는 것이 유용할 수 있습니다.

`LPE_FUSION` 파라미터에서 이에 대한 비트 필드가 있습니다. QGroundControl에서 설정할 수 있는데 "fuse baro"를 체크를 해제합니다.

#### 노이즈 파라미터 튜닝

비젼이나 mocap 데이터의 정확도가 높아서 estimator가 촘촘하게 track해나가길 원한다면, `LPE_VIS_XY` 와 `LPE_VIS_Z` (비젼에 대해서) 혹은 `LPE_VIC_P` (모션 캡쳐에 대해서)와 같은 표준 편차 파라미터를 줄여야 합니다. 이를 줄인다는 것은 estimator가 입력으로 들어오는 pose estimate를 더 신뢰하게 됩니다. 허용하는 최소와 force-save보다 더 낮게 설정해야 합니다.
