# Telemetry
Telemetry can be used to communicate with QGroundControl and is very useful especially for tuning for example as parameters can be changed without plugging in a cable each time.

## 3DR WIFI Telemetry
With the 3DR WIFI Telemetry you just need one transmitter (provided you have a WIFI card/stick in your computer/tablet). Just connect the module to the ```TELEM``` port and it should act as a WIFI station.
```sh
essid: APM_PIX
password: 12345678
```
Once connected to the WIFI, it should automatically connect to QGroundControl.
![](images/hardware/3dr_wifi_1.JPG)
![](images/hardware/3dr_wifi_2.png)
