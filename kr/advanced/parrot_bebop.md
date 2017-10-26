# Bebop 2 - 고급

## FTDI 연결
다음 절차에 따라서 FTDI로 Parrot Bebop 2에 연결합니다.
* 2개 Torx 나사(T5)를 풀어서 앞면 캡을 벗깁니다.
![](../../assets/hardware/bebop/bebop_torx.jpg)
* 핀을 사용해서 ground/RX/TX에 연결하거나 커넥터에 케이블을 납땜합니다.
![](../../assets/hardware/bebop/bebop_serial.jpg)
* FTDI 케이블을 연결하고 실행
```sh
screen /dev/ttyUSB0 115200
```
Bebop에 연결
![](../../assets/hardware/bebop/bebop_ftdi.jpg)
