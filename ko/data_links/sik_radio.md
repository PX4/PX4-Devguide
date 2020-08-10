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
> 3. RFD900x 또는 RFD900ux 텔레메트리 무선 장치에 펌웨어를 업로드하려면 RFD 모뎀 도구를 사용하십시오.

## 빌드 방법

PX4 빌드 툴체인에 기본으로 들어있지 않으므로 8051 컴파일러를 설치해야합니다.

### Mac OS

툴체인을 설치하십시오:

```sh
brew install sdcc
```

표준 SiK 무선 창치 / 3DR 무선 장치 이미지를 빌드하십시오:

```sh
git clone https://github.com/LorenzMeier/SiK.git
cd SiK/Firmware
make install
```

무선 장치에 업로드하십시오 \(**직렬 포트 이름을 바꾸십시오**\):

    tools/uploader.py --port /dev/tty.usbserial-CHANGETHIS dst/radio~hm_trp.ihx
    

## 설정 방법

무선 장치에서는 설정시 AT 명령을 지원합니다.

```sh
screen /dev/tty.usbserial-CHANGETHIS 57600 8N1
```

명령 모드를 시작하십시오:

> **Note** 몇 초 전후동안 절대로 그 어떤 내용도 입력하지 마십시오

    +++
    

현재 설정 목록을 출력하십시오:

    ATI5
    

net ID를 설정한 후 설정 값을 기록하고 무선 장치를 다시 부팅하십시오:

    ATS3=55
    AT&W
    ATZ
    

> **Note** 무선 장치의 전원을 완전히 뽑았다가 다시 연결한 후 두번째 무선 장치에 연결하십시오.