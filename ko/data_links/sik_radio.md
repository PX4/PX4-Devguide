# SiK 무선 통신

[SiK 무선 통신](https://github.com/LorenzMeier/SiK) 텔레메트리 무선 통신용 펌웨어, 도구 모음입니다.

SiK 무선 통신 *활용* 정보는 *PX4 사용자 안내서* [텔레메트리 > SiK 무선 통신](https://docs.px4.io/master/en/telemetry/sik_radio.html)에 있습니다.

("개발자") 정보에서는 소스 코드에서 SiK 펌웨어 빌드 및 AT 명령으로의 설정 방법을 설명합니다.

## 지원 무선 통신 하드웨어

SiK 저장소에서는 다음 텔레메트리 무선 통신 장비용 부트로더 및 펌웨어가 들어있습니다(2020-02-25):

- HopeRF HM-TRP
- HopeRF RF50-DEMO
- RFD900
- RFD900a
- RFD900p
- RFD900pe
- RFD900u
- RFD900ue

> **Note** SiK 저장소에는 현재 RFD900x, RFD900ux 텔레메트리 무선 통신** 펌웨어는 없습니다. 이 무선 통신 장치의 펌웨어를 업데이트(예: MAVLink 2.0 으로의 업데이트)하려면, 다음 절차를 추천드립니다:
> 
> 1. [RFDesign 웹사이트](https://files.rfdesign.com.au/firmware/)에서 적절한 펌웨어를 다운로드하십시오.
> 2. 윈도우 PC에서 [RFD 모뎀 도구](https://files.rfdesign.com.au/tools/)를 다운로드하고 설치하십시오.
> 3. Use the RFD Modem Tools GUI to upload the firmware to your RFD900x or RFD900ux telemetry radio.

## 빌드 방법

You will need to install the required 8051 compiler, as this is not included in the default PX4 Build toolchain.

### Mac OS

Install the toolchain:

```sh
brew install sdcc
```

Build the image for the standard SiK Radio / 3DR Radio:

```sh
git clone https://github.com/LorenzMeier/SiK.git
cd SiK/Firmware
make install
```

Upload it to the radio \(**change the serial port name**\):

    tools/uploader.py --port /dev/tty.usbserial-CHANGETHIS dst/radio~hm_trp.ihx
    

## Configuration Instructions

The radio supports AT commands for configuration.

```sh
screen /dev/tty.usbserial-CHANGETHIS 57600 8N1
```

Then start command mode:

> **Note** DO NOT TYPE ANYTHING ONE SECOND BEFORE AND AFTER

    +++
    

List the current settings:

    ATI5
    

Then set the net ID, write settings and reboot radio:

    ATS3=55
    AT&W
    ATZ
    

> **Note** You might have to power-cycle the radio to connect it to the 2nd radio.