# 초기 설정

PX4에서 개발을 시작하기 전에, 하드웨어가 제대로 셋업되어 있는지 확인하기 위해서 시스템은 디폴트 설정으로 초기화 되어야 합니다. 아래 비디오는 [Pixhawk 하드웨어](../flight_controller/pixhawk.md) 와 [QGroundControl](../qgc/README.md)의 셋업 프로세스를 설명합니다. 지원하는 에어프레임 레퍼런스의 목록은 [여기](../airframes/architecture.md)를 참고하세요.

> **Info** [DAILY BUILD of QGroundControl](https://docs.qgroundcontrol.com/en/releases/daily_builds.html) 다운받기와 아래 비디오를 따라서 여러분의 비행체를 셋업합니다. mission planning, 비행, 파라미터 셋팅에 대해서 보다 상세한 내용을 위해서는 [QGroundControl 튜터리얼](../qgc/README.md)을 참고하세요.

셋업 옵션의 목록은 아래 비디오를 참고하세요.

{% youtube %}https://www.youtube.com/watch?v=91VGmdSlbo4&rel=0&vq=hd720{% endyoutube %}

## 라디오 컨트롤 옵션

PX4 flight stack은 라디오 컨트롤 시스템을 강제하지 않습니다. 비행 모드를 선택하는 개발 스위치에 대해서도 강제사항은 없습니다.

### 라디오 컨트롤 없이 비행

모든 라디오 컨트롤 셋업 체크는 `COM_RC_IN_MODE` 파라미터를 `1`로 설정해서 비활성화시킬 수 있습니다. 이렇게 하면 비행 중인 경우를 제외하고 수동 비행을 허용하지 않습니다.

### 단일 채널 모드 스위치

여러 스위치를 사용하는 대신에 이 모드에서 시스템은 한개 채널만 모드 스위치로 받아들입니다. 이와 관련해서는 [기존 위키](https://pixhawk.org/peripherals/radio-control/opentx/single_channel_mode_switch)를 참고하세요.
