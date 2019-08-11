# Dronecode Platform Hardware/Software Architecture

아래의 그림은 [Dronecode Platform](https://www.dronecode.org/platform/)의 플랜을 보여주는 추상화된 개요를 제공한다. 왼쪽 편은 하나의 하드웨어 설정을 보여줍니다. *flight controller*(밝은 파란색)이 [RTPS](../middleware/micrortps.md)를 통해 *perception computer*(어두운 파랑) 과 연결됩니다. The perception computer provides vision control and object avoidance using a camera sensor array, and has a separate payload camera.

The right hand side of the diagram shows the end-to-end software stack. The stack "approximately" aligns horizontally with the hardware parts of the diagram, and is colour coded to show which software is running on the flight controller and which on the companion computer.

> **Note** The [PX4 Architectural Overview](../concept/architecture.md) provides information about the flight stack and middleware. Offboard APIs are covered in [ROS](../ros/README.md) and [MAVSDK](https://www.dronecode.org/sdk/).

![Dronecode Platform architecture](../../assets/diagrams/dronecode_platform_architecture.jpg)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/14sgSpcs7NcBatW-qn0dLtyMHvwNMSSlm/view?usp=sharing. Request access from dev team. -->