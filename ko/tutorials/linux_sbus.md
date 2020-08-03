# 리눅스에 리모콘 수신기 연결 (S.Bus 포함)

이 절에서는 [지원 리모콘 수신기](https://docs.px4.io/master/en/getting_started/rc_transmitter_receiver.html)를 임의의 직렬 포트에 연결하고 사용할 목적으로 리눅스 기반 오토파일럿을 설정하는 방법을 알려드립니다.

S.Bus에 비해 원격 조종 타입은 수신기를 직렬 포트에 연결하거나 USB to TTY 직렬 케이블(PL2302 USB to Serial TTL 변환기)로 USB에 연결할 수 있습니다.

> **Note** S.Bus 수신기(또는 후타바, 래디오링크 등의 인코더)를 사용하는 목적이라면 보통 [신호 반전 회로](#signal_inverter_circuit)를 거쳐 수신기와 장치를 연결해야겠지만, 이외의 경우 설정은 동일합니다.

이후, 장치에서 다음과 같이 [PX4 원격 조정 드라이버를 시작](#start_driver)하십시오.

## 드라이버 시작 {#start_driver}

To start the RC driver on a particular UART (e.g. in this case `/dev/ttyS2`):

    rc_input start -d /dev/ttyS2
    

For other driver usage information see: [rc_input](../middleware/modules_driver.md#rcinput).

## Signal Inverter Circuit (S.Bus only) {#signal_inverter_circuit}

S.Bus is an *inverted* UART communication signal.

While some serial ports/flight controllers can read an inverted UART signal, most require a signal inverter circuit between the receiver and serial port to un-invert the signal.

> **Tip** This circuit is also required to read S.Bus remote control signals through the serial port or USB-to-TTY serial converter.

This section shows how to create an appropriate circuit.

### Required Components

* 1x NPN transistor (e.g. NPN S9014 TO92)
* 1x 10K resistor
* 1x 1K resistor

> **Note** Any type/model of transistor can be used because the current drain is very low.

### Circuit Diagram/Connections

Connect the components as described below (and shown in the circuit diagram):

* S.Bus signal &rarr; 1K resistor &rarr; NPN transistor base
* NPN transistor emit &rarr; GND
* 3.3VCC &rarr; 10K resistor &rarr; NPN transistor collection &rarr; USB-to-TTY rxd
* 5.0VCC &rarr; S.Bus VCC
* GND &rarr; S.Bus GND

![Signal inverter circuit diagram](../../assets/driver_sbus_signal_inverter_circuit_diagram.png)

The image below shows the connections on a breadboard.

![Signal inverter breadboard](../../assets/driver_sbus_signal_inverter_breadboard.png)