# 자주 묻는 문제


## 빌드 에러

### Flash Overflow

> **Tip** FMUv4 아키텍쳐는 사용해서 플래쉬 용량의 2배를 가집니다. [Pixracer](http://dev.px4.io/hardware-pixracer.html)가 적용되는 첫번째 세대의 보드입니다.

보드로 로드할 수 있는 코드양은 플래쉬 메모리의 크기에 따라 제한됩니다. 모듈을 추가하거나 코드를 추가하는 경우 플래쉬 메모리를 초과하면 결국 "flash overflow"가 발생하게 됩니다. 업스트림 버전은 항상 빌드가 되지만 개발자가 로컬로 작업하면서 추가하게 되면 overflow가 발생할 수 있습니다.

```sh
region `flash' overflowed by 12456 bytes
```

이를 해결하는 방법은 최신 하드웨어를 사용하거나 불필요한 모듈을 제거하고 빌드하는 것입니다. 빌드 설정은 [여기](https://github.com/PX4/Firmware/tree/master/cmake/configs)에 저장되어 있습니다. 모듈을 제거하기 위한 코멘트 처리 방법 :

<div class="host-code"></div>

```cmake
#drivers/trone
```

## USB 에러

### upload가 계속 실패하는 경우

Ubuntu에서 modem manager를 삭제합니다. 삭제 방법은 :

```sh
sudo apt-get remove modemmanager
```
