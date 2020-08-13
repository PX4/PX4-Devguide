# UAVCAN 소개

![UAVCAN 로고](../../assets/uavcan-logo-transparent.png)

[UAVCAN](http://uavcan.org)는 보드상에서 처리하는 네트워크로 오토파일럿을 항공 전자장비에 연결할 수 있게 합니다. 다음 하드웨어를 지원합니다:

* 모터 컨트롤러
  
  * [Zubax Orel 20](https://zubax.com/product/zubax-orel-20)
    
    > **Note** [Sapog Firmware](https://github.com/px4/sapog)(오픈소스)를 실행합니다. [Sapog Reference Hardware](https://github.com/PX4/Hardware/tree/master/sapog_reference_hardware)에 기반합니다.

* 대기속도 센서
  
  * [Thiemar airspeed sensor](https://github.com/thiemar/airspeed)
* GPS 및 GLONASS용 GNSS 리시버 
  * [Zubax GNSS](https://zubax.com/products/gnss_2)
* 전력량계 
  * [Pomegranate Systems Power Module](http://docs.px4.io/master/en/power_module/pomegranate_systems_pm.html)
  * [CUAV CAN PMU Power Module](http://docs.px4.io/master/en/power_module/cuav_can_pmu.html)

In contrast to hobby-grade devices it uses rugged, differential signalling and supports firmware upgrades over the bus. All motor controllers provide status feedback and implement field-oriented-control \(FOC\).

> **Note** PX4 requires an SD card for UAVCAN node allocation and firmware upgrade. It is not used during flight by UAVCAN.

## 초기 설정

The following instructions provide a step-by-step guide to connect and setup a quadcopter with ESCs and GPS connected via UAVCAN. The hardware of choice is a Pixhawk 2.1, Zubax Orel 20 ESCs and a Zubax GNSS GPS module.

### 결선

The first step is to connect all UAVCAN enabled devices with the flight controller. The following diagram displays how to wire all components. The used Zubax devices all support a redundant CAN interface in which the second bus is optional but increases the robustness of the connection.

![UAVCAN 결선](../../assets/UAVCAN_wiring.png)

It is important to mention that some devices require an external power supply \(e.g. Zubax Orel 20\) and others can be powered by the CAN connection \(e.g Zubax GNSS\) itself. Please refer to the documentation of your hardware before continuing with the setup.

### 펌웨어 설치

Next, follow the instructions in [UAVCAN Configuration](../uavcan/node_enumeration.md) to activate the UAVCAN functionalities in the firmware. Disconnect your power supply and reconnect it. After the power cycle all UAVCAN devices should be detected which is confirmed by a beeping motor on the Orel 20 ESCs. You can now continue with the general setup and calibration.

Depending on the used hardware, it can be reasonable to perform an update of the firmware on the UAVCAN devices. This can be done via the UAVCAN itself and the PX4 firmware. For more details please refer to the instructions in [UAVCAN Firmware](../uavcan/node_firmware.md).

## 노드 펌웨어 업그레이드

PX4 미들웨어에서는 UAVCAN 노드에 일치하는 펌웨어를 받았을 경우 자동으로 업그레이드합니다. 절차와 요구사항은 [UAVCAN 펌웨어](../uavcan/node_firmware.md) 페이지에 있습니다.

## 모터 컨트롤러 기수 부여 및 설정

ID와 각 모터 컨트롤러의 회전 방향은 간단한 설치 루틴에서 설치 과정을 마친 후 할당할 수 있습니다: [UAVCAN 노드 기수 부여](../uavcan/node_enumeration.md). QGroundControl에서 사용자가 루틴 실행을 시작할 수 있습니다.

## 참고할만한 링크

* [홈페이지](http://uavcan.org)
* [상세사양](https://uavcan.org/specification/)
* [구현 및 자습서](http://uavcan.org/Implementations)