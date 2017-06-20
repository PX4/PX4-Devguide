# 여러 가지 노트

여기에는 UAVAN을 셋업하고 동작할때 문제를 해결하기 위해서 방법들을 모아두었습니다.

### arm은 되었으나 모터가 회전하지 않는 경우

PX4 펌웨어가 arm이지만 모터가 회전하지 않는다면 **UAVCAN\_ENABLE** 파라미터를 체크하세요. 출력으로 UAVCAN을 통해 연결된 ESC를 사용하기 위해서 3으로 설정해야만 합니다. 더우기 thrust가 증가하기 전에 모터가 회전하지 않는다면 **UAVCAN\_ESC\_IDLT** 을 체크하고 1로 설정하세요.

### Zubax Babel로 디버깅하기

UAVCAN 버스에서 트랜지션을 디버깅하기에 훌륭한 도구는 [Zubax Babel](https://docs.zubax.com/zubax_babel)로 [GUI 도구](http://uavcan.org/GUI_Tool/Overview/)와 결합되어 있습니다. node를 테스트하거나 수동으로 UAVCAN이 활성화된 ESC를 제어하는데 Pixhawk 하드웨어와 독립되어서 사용할 수도 있습니다.
