# UAVCAN 기수 부여 및 설정

> **Note** 아래와 같이 'Enable UAVCAN' 확인 상자에 표시하여 기본 모터 출력 버스로서 UAVCAN을 활성화하십시오. 또는, *QGroundControl* 매개변수 편집기에서 UAVCAN_ENABLE 매개변수를 '3'으로 설정할 수 있습니다. '2'로 설정하면 CAN을 가동할 수 있으나, PWM 모터 출력 설정이 남습니다.

[QGroundControl](../qgc/README.md)을 활용하여 설정 보기로 전환하십시오. 좌측의 전원 설정을 선택하십시오. 'start assignment' 단추를 누르십시오.

처음 비프음이 울린 다음에는 처음 ESC의 프로펠러를 올바른 회전 방향으로 재빨리 돌리십시오. The ESCs will all beep each time one is enumerated. Repeat this step for all motor controllers in the order as shown on the [motor map](../airframes/airframe_reference.md). ESCs running the Sapog firmware will need to be rebooted after enumeration for the new enumeration ID to be applied. This step has to be performed only once and does not need to be repeated after firmware upgrades.

![UAVCAN Enumeration Controls (bottom right of image)](../../assets/uavcan-qgc-setup.png)