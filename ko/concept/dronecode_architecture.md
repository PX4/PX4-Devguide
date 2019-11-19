# PX4 Platform Hardware/Software Architecture

The diagram below provides a forward-looking high level overview of a full PX4 onboard and offboard stack. 왼쪽 편은 하나의 하드웨어 설정을 보여줍니다. *flight controller*(밝은 파란색)이 [RTPS](../middleware/micrortps.md)를 통해 *perception computer*(어두운 파랑) 과 연결됩니다. Perception computer는 카메라 센서 모음을 활용한 비전 컨트롤과 물체 회피를 제공하고 독립된 페이로드 카메라를 갖고 있습니다.

오른쪽은 풀스택의 소프트웨어를 보여줍니다. The stack "approximately" aligns horizontally with the hardware parts of the diagram, and is colour-coded to show which software is running on the flight controller and which on the companion computer.

> **Note** [PX4 Architectural Overview](../concept/architecture.md) 는 flight stack과 middleware에 대한 정보를 제공합니다. Offboard APIs are covered in [ROS](../ros/README.md) and [MAVSDK](https://mavsdk.mavlink.io/develop/en/index.html).

![PX4 Platform architecture](../../assets/diagrams/dronecode_platform_architecture.jpg)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/14sgSpcs7NcBatW-qn0dLtyMHvwNMSSlm/view?usp=sharing. Request access from dev team. -->