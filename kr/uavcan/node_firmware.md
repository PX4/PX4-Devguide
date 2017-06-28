# UAVCAN 펌웨어 업그레이드

## Vectorcontrol ESC 코드베이스 (Pixhawk ESC 1.6와 S2740VC)

ESC 코드 다운로드 :

<div class="host-code"></div>

```sh
git clone https://github.com/thiemar/vectorcontrol
cd vectorcontrol
```

### UAVCAN Bootloader 플래쉬하기

UAVCAN으로 펌웨어를 업데이트하기 전에, Pixhawk ESC 1.6은 UAVCAN bootloader가 플래쉬되어 있어야 합니다. bootloader를 빌드하기 위해서 다음을 실행:

<div class="host-code"></div>

```sh
make clean && BOARD=px4esc_1_6 make -j8
```

빌드를 마치고 bootloader 이미지는 `firmware/px4esc_1_6-bootloader.bin`에 위치하며 OpenOCD 설정은 `openocd_px4esc_1_6.cfg`에 위치합니다. ESC에서 bootloader를 설치하기 위해서 [다음 지시](../uavcan/bootloader_installation.md)를 따릅니다.

### 메인 바이너리 컴파일하기

<div class="host-code"></div>

```sh
BOARD=s2740vc_1_0 make && BOARD=px4esc_1_6 make
```

이렇게 하면 지원하는 ESC에 대해서 UAVCAN 노드 펌웨어를 빌드합니다. 펌웨어 이미지는 `com.thiemar.s2740vc-v1-1.0-1.0.<git hash>.bin` 와 `org.pixhawk.px4esc-v1-1.6-1.0.<git hash>.bin`에 위치합니다.

## Sapog 코드베이스 (Pixhawk ESC 1.4와 Zubax Orel 20)

Sapog 코드베이스 다운로드:

<div class="host-code"></div>

```sh
git clone https://github.com/PX4/sapog
cd sapog
git submodule update --init --recursive
```

### UAVCAN Bootloader 플래쉬하기

UAVCAN을 통해서 펌웨어를 업데이트하기 전에, ESC는 플래쉬를 위해 UAVCAN bootloader를 필요로 합니다. bootloader는 다음과 같이 빌드할 수 있습니다:

<div class="host-code"></div>

```sh
cd bootloader
make clean && make -j8
cd ..
```

bootloader 이미지는 `bootloader/firmware/bootloader.bin`에 위치하며 OpenOCD 설정은 `openocd.cfg`에 위치합니다. ESC에 bootloader를 설치하기 위해서는 [다음 지시](../uavcan/bootloader_installation.md) 따릅니다.

### 메인 바이너리 컴파일하기

<div class="host-code"></div>

```sh
cd firmware
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```
GCC의 최근 버전은 링크하는 동안 세그먼트폴트가 발생할 수 있습니다. 작성하는 당시에 4.9 버전으로 작업했습니다. 펌웨어 이미지는 `firmware/build/io.px4.sapog-1.1-1.7.<xxxxxxxx>.application.bin`에 위치하며 `<xxxxxxxx>`는 임의의 숫자나 문자가 됩니다. Zubax Orel 20 (1.0과 1.1)의 2가지 하드웨어 버전이 있습니다. 바이너리를 올바른 폴더에 복사했는지 확인합니다. ESC 펌웨어는 하드웨어 버전을 검사하고 양쪽 모든 제품에서 동작합니다.

## Zubax GNSS

[프로젝트 페이지](https://github.com/Zubax/zubax_gnss)를 참고해서 펌웨어를 빌드하고 플래쉬하는 방법을 익힐 수 있습니다.
Zubax GNSS는 UAVCAN이 가능한 bootloader와 함께 출시되며 펌웨어는 UAVCAN을 통해 아래에서 설명한 동일 방식으로 업데이트할 수 있습니다.

## autopilot에서 펌웨어 설치

UAVCAN node 파일 이름은 Pixhawk가 네트워크에 있는 모든 UAVCAN 장치를 업데이트할 수 있도록 이름 규칙을 따릅니다. 제조사와 상관이 없습니다. 위에 설정한 단계에서 생성된 펌웨어 파일은 반드시 적절한 SD카드나 PX4 ROMFS에 복사되어야 장치에 업데이트가 가능합니다.

펌웨어 이미지 이름에 대한 규칙은:

  ```<uavcan name>-<hw version major>.<hw version minor>-<sw version major>.<sw version minor>.<version hash>.bin```

  e.g. ```com.thiemar.s2740vc-v1-1.0-1.0.68e34de6.bin```

하지만 공간/성능 제약으로(이름은 28 캐릭터를 초과할 수 없음) UAVCAN 펌웨어 업데이트는 이 파일이름을 다음과 같은 디렉토리 구조로 나눠서 저장합니다.:

  ```/fs/microsd/fw/<node name>/<hw version major>.<hw version minor>/<hw name>-<sw version major>.<sw version minor>.<git hash>.bin```

 e.g.
 ```
 s2740vc-v1-1.0.68e34de6.bin
 /fs/microsd/fw/io.px4.sapog/1.1/sapog-1.7.87c7bc0.bin
 ```

ROMFS기반 업데이터는 이런 패턴을 따릅니다. 하지만 파일이름에 ```_``` 접두어를 붙여서 펌웨어를 다음에 추가합니다 :

  ```/etc/uavcan/fw/<device name>/<hw version major>.<hw version minor>/_<hw name>-<sw version major>.<sw version minor>.<git hash>.bin```

## 바이너리를 PX4 ROMFS에 넣기

최종 결과 파일 위치는 :

  * S2740VC ESC: `ROMFS/px4fmu_common/uavcan/fw/com.thiemar.s2740vc-v1/1.0/_s2740vc-v1-1.0.<git hash>.bin`
  * Pixhawk ESC 1.6: `ROMFS/px4fmu_common/uavcan/fw/org.pixhawk.px4esc-v1/1.6/_px4esc-v1-1.6.<git hash>.bin`
  * Pixhawk ESC 1.4: `ROMFS/px4fmu_common/uavcan/fw/org.pixhawk.sapog-v1/1.4/_sapog-v1-1.4.<git hash>.bin``
  * Zubax GNSS v1: `ROMFS/px4fmu_common/uavcan/fw/com.zubax.gnss/1.0/gnss-1.0.<git has>.bin`
  * Zubax GNSS v2: `ROMFS/px4fmu_common/uavcan/fw/com.zubax.gnss/2.0/gnss-2.0.<git has>.bin`

ROMFS/px4fmu_common 디렉토리는 Pixhawk에 있는 /etc로 마운트된다는 점에 유의하세요.

### 펌웨어 업그레이드 프로세스 시작

[PX4 Flight Stack](../concept/flight_stack.md)을 사용할때, UAVCAN을 'Power Config' 섹션에 활성화시키고 UAVCAN 펌웨어 업그레이드를 시도하기 전에 시스템을 리부팅합니다.

선택적으로 UAVCAN 펌웨어 업그레이드는 NSH를 통해 수동으로 시작할 수 있습니다:

```sh
uavcan start
uavcan start fw
```
