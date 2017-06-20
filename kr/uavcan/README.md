# UAVCAN 소개

![](../../assets/uavcan-logo-transparent.png)

[UAVCAN](http://uavcan.org)은 온보드 네트워크로 autopilot이 항공전자기기에 연결하는 것을 허용합니다. 다음과 같은 하드웨어를 지원:

* 모터 컨트롤러
  * [Pixhawk ESC](https://pixhawk.org/modules/pixhawk_esc)
  * [SV2740 ESC](https://github.com/thiemar/vectorcontrol)
  * [Zubax Orel 20](https://zubax.com/product/zubax-orel-20)
* Airspeed 센서
  * [Thiemar airspeed sensor](https://github.com/thiemar/airspeed)
* GNSS 수신기로 GPS와 GLONASS용
  * [Zubax GNSS](http://zubax.com/product/zubax-gnss)

취미레벨의 장치와 구분되게 별도의 신호를 사용하고 버스 상에서 펌웨어 업그레이드를 지원합니다. 모든 모터 컨트롤러는 상태 피드백을 제공하며 필드-기반-컨트롤\(FOC\)을 구현합니다.

## 초기 셋업

다음에 지시하는 내용은 단계별로 쿼드콥터를 ESC와 GPS에 연결하고 셋업하는 방법을 가이드합니다. 선택한 하드웨어는 Pixhawk 2.1, Zubax Orel20 ESC 그리고 Zubax GNSS GPS 모듈입니다.

### 전선연결

첫번째 단계는 비행 제어기와 모든 활성화된 UAVCAN를 연결하는 것입니다. 다음 다이어그램에서는 모든 컴포넌트를 연결하는 방법을 보여줍니다. 사용한 Zubax 장치들 모두 여분의 CAN 인터페이스를 지원하며 두번째 버스는 선택사항으로 연결의 안정성을 증가시킵니다.

![](../../assets/UAVCAN_wiring.png)

일부 장치는 외부 전원 공급을\(예로 Zubax Orel 20\) 필요로 하며 다른 장치들은 CAN 연결에서 전원을\(예로 Zubax GNSS\) 공급받습니다. 셋업을 하기 전에 하드웨어 문서를 참고하세요.

### 펌웨어 셋업

다음으로 [UAVCAN 설정](../uavcan/node_enumeration.md)에 있는 지시를 따라서 펌웨어에 있는 UAVCAN 기능을 활성화시킵니다. 여러분의 전원 공급 장치 연결을 해제하고 다시 연결합니다. 파워 사이클 후에, 모든 UAVCAN 장치는 Orel 20 ESC에서 비프임이 나는 모터로 어떤 것이 확인여부를 검출해 내야 합니다. 일반 셋업과 칼리브레이션을 계속 진행합니다.

사용한 하드웨어에 따라서 UAVCAN 장치에 있는 펌웨어의 업데이트를 수행하는 것이 자연스럽습니다. 이는 UAVCAN 자신과 PX4 펌웨어를 통해서 진행됩니다. 보다 자세한 내용은 [UAVCAN 펌웨어](../uavcan/node_firmware.md)에 있는 내용을 참고하세요.

## Node 펌웨어 업그레이드

매칭하는 펌웨어가 제공되는 경우, PX4 미들웨어는 자동으로 UAVCAN 노드에 있는 펌웨어를 업그레이드합니다. 절차와 요구사항은 [UAVCAN 펌웨어](../uavcan/node_firmware.md) 페이지를 참고하세요.

## 모터 컨트롤러 열거와 설정

ID와 각 모터 컨트롤러의 회전 방향은 간단한 셋업 루틴에 따라 설치한 후에 할당할 수 있습니다 : [UAVCAN Node 열거](../uavcan/node_enumeration.md). 해당 루틴은 QGroundControl로 구동시킬 수 있습니다.

## 유용한 링크

* [홈페이지](http://uavcan.org)
* [스펙](http://uavcan.org/Specification)
* [구현과 튜토리얼](http://uavcan.org/Implementations)
