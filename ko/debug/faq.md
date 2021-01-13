!REDIRECT "https://docs.px4.io/master/ko/debug/faq.html"

# 자주 묻는 질문

## 빌드 오류

### 플래시 오버플로우

> **Tip** 플래시 용량을 두배로 활용하려면 FMUv4 아키텍처를 활용하십시오. 이 세대에서 가장 먼저 나온 보드는 [픽스레이서](https://docs.px4.io/master/en/flight_controller/pixracer.html)입니다.

보드에서 불러올 수 있는 코드는 플래시 메모리 양에 제한적입니다. 추가 모듈 또는 코드를 넣을 떄 플래시 메모리의 용량을 넘길 수 있습니다. 이 결과로 "플래시 오버플로우"가 일어납니다. 업스트림 버전을 늘 빌드하겠지만, 개발자가 무얼 추가하느냐에 따라 로컬에서 오버플로우가 일어날 수 있습니다.

```sh
region `flash' overflowed by 12456 bytes
```

이 문제를 해결하려면, 최신 하드웨어를 활용하거나 빌드에서 활용상 별로 중요하지 않은 모듈을 빼십시오. The configuration is stored in **/PX4-Autopilot/boards/px4** (e.g. [PX4-Autopilot/boards/px4/fmu-v5/default.cmake](https://github.com/PX4/PX4-Autopilot/blob/master/boards/px4/fmu-v5/default.cmake)). 모듈을 제거하려면 다음과 같이 주석처리하시면 됩니다:

```cmake
#tune_control
```

## USB 오류

### 업로드를 전혀 끝낼 수 없음

우분투의 경우 modemmanager를 제거하십시오:

```sh
sudo apt-get remove modemmanager
```