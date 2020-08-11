# PX4 플랫폼 하드웨어/소프트웨어 구조

아래의 그림은 PX4 보드 내외의 전체 스택 개요를 고수준 관점에서 보여드립니다. 왼쪽 편은 가능한 하나의 하드웨어 설정을 보여줍니다. *flight controller*(밝은 파란색)부분을 [RTPS](../middleware/micrortps.md)로 *perception computer*(어두운 파랑)과 연결합니다. 인지기능을 지닌 컴퓨터(Perception Computer)는 카메라 센서 배열을 활용한 시각 제어와 물체 회피 기능을 부여하며, 별도의 페이로드 카메라를 갖추고 있습니다.

그림의 오른편에는 종단간 소프트웨어 스택을 보여줍니다. 스택은 그림의 하드웨어 부분에 맞춰 "거의" 수직으로 줄을 맞추었으며, 어떤 프로그램에 비행체 컨트롤러에서 실행하고 어떤 프로그램이 보조 컴퓨터에 들어가는지 색상으로 구분하여 보여주고 있습니다.

> **Note** [PX4 구조 개요](../concept/architecture.md) 는 flight stack과 middleware에 대한 정보를 제공합니다. Offboard API는 [ROS](../ros/README.md) 와 [MAVSDK](https://mavsdk.mavlink.io/develop/en/index.html)에서 다룹니다.

![PX4 플랫폼 구조](../../assets/diagrams/dronecode_platform_architecture.jpg)

<!-- The drawing is on draw.io: https://drive.google.com/file/d/14sgSpcs7NcBatW-qn0dLtyMHvwNMSSlm/view?usp=sharing. Request access from dev team. -->