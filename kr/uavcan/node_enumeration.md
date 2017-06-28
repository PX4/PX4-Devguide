# UAVCAN 열거와 설정

> **Note** 아래에서 보는 바와 'Enable UAVCAN' 체크박스를 체크해서 디폴트 모터 출력 버스로 UAVCAN을 활성화시킬 수 있습니다. 대안으로 UAVCAN ENABLE 파라미터를 QGroundControl 파라미터 편집기에서 '3'으로 설정할 수 있습니다. '2'로 설정하면 CAN이 활성화되지만 모터 출력은 PWM을 유지합니다.

[QGroundControl](../qgc/README.md)를 사용하고 Setup view로 전환합니다. 왼쪽에 Power Configuration을 선택합니다. 'start assignment' 버튼을 클릭합니다.

첫번째 비프음 후에, 첫번째 ESC에 있는 프로펠러가 잽싸게 올바른 방향으로 회전합니다. ESC는 모두 비프음이 나고 한 번에 하나만 열거됩니다. [motor map](../airframes/airframe_reference.md)에서 보는바와 같이 순서대로 모든 모터 제어기에 대해서 이 단계를 되풀이하게 됩니다. 이 단계는 한 번만 수행되고 펌웨어 업그레이드 후에는 반복할 필요가 없습니다.

![UAVCAN 열거 컨트롤 (이미지의 바닥 오른쪽)](../../assets/uavcan-qgc-setup.png)
