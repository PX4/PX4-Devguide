!REDIRECT "https://docs.px4.io/master/ko/robotics/"

# 로보틱스

로보틱스 API는 PX4를 외부의 플라이트 스택 처리 환경(비행체 제어 장치)에서 [보조 컴퓨터](../companion_computer/pixhawk_companion.md) 또는 기타 컴퓨터 환경에서 PX4를 제어할 수 있게 합니다. 이 API는 [MAVLink](../middleware/mavlink.md)나 [RTPS](../middleware/micrortps.md)로 PX4와 통신합니다.

PX4는 [MAVSDK](https://www.dronecode.org/sdk/)와 [ROS](../ros/README.md)가 들어간 로보틱스 API와 함께 활용할 수 있습니다. [DroneKit](../robotics/dronekit.md) 도 활용할 수 있지만, PX4 활용에 최적화하진 않았습니다.