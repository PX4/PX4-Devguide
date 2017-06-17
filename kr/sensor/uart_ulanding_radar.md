# uLanding Radar

uLanding radar는 [Aerotenna](http://aerotenna.com/sensors/) 제품으로 대상에 대한 거리를 측정하는데 사용합니다.


## 하드웨어에 대한 드라이버를 활성화
현재 이 radar 장치는 OS NuttX를 실행하는 하드웨어에서 지원하며 인터페이스에 대한 시리얼 포트를 제공합니다. 일부 하드웨어의 경우 플래쉬 공간이 작으므로 타겟 장치에 대해서 드라이버를 빌드할 수 있도록 해야합니다.
이렇게 하기 위해서는 빌드하고자 하는 타겟에 관련된 cmake config 파일에 다음 라인을 추가합니다:
```
drivers/ulanding
```

모든 config 파일은 [여기](https://github.com/PX4/Firmware/tree/master/cmake/configs)에 위치하고 있습니다.

## 드라이버 구동시키기
sytem이 시작되는 동안 radar를 위해 driver를 구동시키라고 sytem에게 알려야만 합니다.
SD 카드에 위치한 [extras.txt](../advanced/system_startup.md) 파일에 간단하게 다음 라인을 추가합니다.
```
ulanding_radar start /dev/serial_port
```

위에 명령에서 마지막 인자는 여러분이 하드웨어를 연결할 시리얼 포트로 교체해야만 합니다.
만약 여러분이 특정 포트를 지정하지 않으면 드라이버는 /dev/ttyS2를 사용하며 Pixhawk에서는 TELEM2 포트를 사용하게 됩니다.

**경고**

radar 장치를 TELEM2에 연결하고 나서 SYS_COMPANION 파라미터를 0으로 설정했는지 확인합니다. 그렇지 않으면 시리얼 포트는 다른 어플리케이션에서 사용될 수 있으며 예상치 못한 동작을 할 수도 있습니다.
