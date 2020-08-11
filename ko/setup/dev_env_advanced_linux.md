# 고급 리눅스 설치 활용 사례

## JTAG 프로그래밍 어댑터 활용

JTAG 프로그래밍 어댑터 사용을 목적으로 USB 버스에 분명히 접근할 수 있어야 할 때가 있습니다.

> **Note** 아치 리눅스에서는 다음에 나오는 모든 명령에서 plugdev 그룹을 uucp 그룹으로 바꾸십시오.

`sudo` 실행 상태에서 `ls`명령을 그냥 실행하면 아래 명령의 존재를 제대로 확인할 수 있습니다:

```sh
sudo ls
```

그 다음 `sudo` 명령으로 임시로 권한을 얻어 다음 명령을 실행하십시오:

```sh
cat > $HOME/rule.tmp <<_EOF
# All 3D Robotics (includes PX4) devices
SUBSYSTEM=="usb", ATTR{idVendor}=="26AC", GROUP="plugdev"
# FTDI (and Black Magic Probe) Devices
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", GROUP="plugdev"
# Olimex Devices
SUBSYSTEM=="usb",  ATTR{idVendor}=="15ba", GROUP="plugdev"
_EOF
sudo mv $HOME/rule.tmp /etc/udev/rules.d/10-px4.rules
sudo /etc/init.d/udev restart
```

사용자를 **plugdev** 그룹에 추가해야 합니다(아치 리눅스 사용자는 uucp 그룹에 추가해야 합니다).

```sh
sudo usermod -a -G plugdev $USER
```