# Dronecode Platform Hardware/Software Architecture

아래의 그림은 [Dronecode Platform](https://www.dronecode.org/platform/)의 플랜을 보여주는 추상화된 개요를 제공합니다. 왼쪽 편은 하나의 하드웨어 설정을 보여줍니다. *flight controller*(밝은 파란색)이 [RTPS](../middleware/micrortps.md)를 통해 *perception computer*(어두운 파랑) 과 연결됩니다. Perception computer는 카메라 센서 모음을 활용한 비전 컨트롤과 물체 회피를 제공하고 독립된 페이로드 카메라를 갖고 있습니다.

오른쪽은 풀스택의 소프트웨어를 보여줍니다. 스택은 거의 하드웨어의 부품들과 수평적으로 정렬되어 있습니다. 그리고 어떤 소프트웨어가 flight controller와 companion computer에서 수행되는지 구분하기위해 색상으로 구분되어있습니다.

> **Note** [PX4 Architectural Overview](../concept/architecture.md) 는 flight stack과 middleware에 대한 정보를 제공합니다. Offboard API는 [ROS](../ros/README.md) 와 [MAVSDK](https://www.dronecode.org/sdk/)에서 담당합니다.

![Dronecode Platform architecture](../../assets/diagrams/dronecode_platform_architecture.jpg)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/14sgSpcs7NcBatW-qn0dLtyMHvwNMSSlm/view?usp=sharing. Request access from dev team. -->