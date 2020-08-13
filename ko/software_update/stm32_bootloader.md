# STM32 부트로더

PX4 부트로더 코드는 깃허브 [Bootloader](https://github.com/px4/bootloader)에 있습니다.

## 지원 보드

* FMUv2 (Pixhawk 1, STM32F4)
* FMUv3 (Pixhawk 2, STM32F4)
* FMUv4 (Pixracer 3 and Pixhawk 3 Pro, STM32F4)
* FMUv5 (Pixhawk 4, STM32F7)
* TAPv1 (TBA, STM32F4)
* ASCv1 (TBA, STM32F4)

## 부트로드 빌드하기

```bash
git clone https://github.com/PX4/Bootloader.git
cd Bootloader
git submodule init
git submodule update
make
```

위 과정을 거치면 Bootloader 디렉토리내에 지원되는 모든 보드에 대한 elf파일이 생성됩니다.

## 부트로더 플래싱하기

> **Warning** 몇몇 보드의 경우 JTAG / SWD 접속을 위해서는 올바른 파워 시퀀스가 매우 중요합니다. 아래 단계를 설명한 것과 같이 정확하게 수행하십시오.

다음 안내는 Blackmagic / Dronecode 프로브를 사용하는 경우에 유효합니다. 다른 JTAG 프로브는 유사하지만 다른 절차가 필요합니다. 부트로더를 플래싱하는 개발자는 반드시 관련된 지식을 숙지하고 있어야 합니다. 이를 어떻게 하는지를 모르는 경우에는 부트로더와 관련된 무언가를 변경하는 것이 정말로 필요한지 다시 한번 고려해보시기 바랍니다.

절차는 다음과 같습니다.

1. JTAG 케이블 연결 제거
2. USB 전원 케이블 연결
3. JTAG 케이블 연결

### Black Magic / Dronecode 프로브

#### 올바른 시리얼 포트 사용

* LINUX: `/dev/serial/by-id/usb-Black_Sphere_XXX-if00`
* MAC OS: 반드시 tty.xxx 포트가 아닌 cu.xxx 포트 사용할 것 : `tar ext /dev/tty.usbmodemDDEasdf`

```bash
arm-none-eabi-gdb
  (gdb) tar ext /dev/serial/by-id/usb-Black_Sphere_XXX-if00
  (gdb) mon swdp_scan
  (gdb) attach 1
  (gdb) mon option erase
  (gdb) mon erase_mass
  (gdb) load tapv1_bl.elf
        ...
        Transfer rate: 17 KB/sec, 828 bytes/write.
  (gdb) kill
```

### J-Link

[J-Link GDB server](https://www.segger.com/jlink-gdb-server.html)관련 안내입니다.

#### 준비사항

Segger 웹사이트의 [Download the J-Link software](https://www.segger.com/downloads/jlink)의 안내사항을 따라 다운로드 및 설치를 수행하십시오.

#### JLink GDB 서버 실행

다음 명령어는 STM32F427VI SoC 기반의 비행 제어기용 서버를 실행하기 위해 사용됨: 

```bash
JLinkGDBServer -select USB=0 -device STM32F427VI -if SWD-DP -speed 20000
```

일반적인 타겟의 `--device`/SoC 옵션:

* **FMUv2, FMUv3, FMUv4, aerofc-v1, mindpx-v2:** STM32F427VI
* **px4_fmu-v4pro:** STM32F469II
* **px4_fmu-v5:** STM32F765II
* **crazyflie:** STM32F405RG

#### GDB 연결

```bash
arm-none-eabi-gdb
  (gdb) tar ext :2331
  (gdb) load aerofcv1_bl.elf
```

### 트러블슈팅

위의 명령어가 존재하지 않는 경우에는, Blackmagic 프로브를 사용하지 않는 경우이거나 소프트웨어가 업데이트되지 않은 경우입니다. 프로브의 소프트웨어를 먼저 업그레이드 하시기 바랍니다.

아래의 에러 메시지가 발생하는 경우:

    Error erasing flash with vFlashErase packet
    

(JTAG 연결은 그대로 유지하고) 타겟 연결을 제거하고 아래를 실행합니다.

```bash
mon tpwr disable
swdp_scan
attach 1
load tapv1_bl.elf
```

이 절차는 타겟의 파워를 비활성화하고 플래싱 과정이 재시도합니다.