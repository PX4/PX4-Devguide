---
translated_page: https://github.com/PX4/Devguide/blob/master/en/airframes_plane/wing_z_84.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# Wing Wing Z-84 Airframe

## 部件列表

> **提示：** 确认订购的是PNF版，该版本包含电机，螺旋桨和电调。Kit版需要单独购买以上部件。


- Zeta Science Wing-Wing Z-84 *PNF* ([Hobbyking Store](http://hobbyking.com/hobbyking/store/RC_PRODUCT_SEARCH.asp?strSearch=z-84))
- 1800 mAh 2S LiPo
  - [Team Orion 1800mAh 7.4V 50C 2S1P](https://www.brack.ch/team-orion-1800mah-7-4v-50c-315318)
- FrSky D4R-II接收机或同类产品（根据手册设置跳帽为PPM输出）
  - [Pixracer kit](https://docs.px4.io/en/flight_controller/pixracer.html) (包含GPS和电源模块)
  - [Mini telemetry set](https://docs.px4.io/en/flight_controller/pixfalcon.html#availability) for HKPilot32
  - [电子空速传感器](https://docs.px4.io/en/flight_controller/pixfalcon.html#availability) for HKPilot32 / Pixfalcon
  - 备用零件
    - [O-Rings螺旋桨保护环](http://www.hobbyking.com/hobbyking/store/__27339__Wing_Wing_Z_84_O_Ring_10pcs_.html)
  - [备用螺旋桨](http://www.hobbyking.com/hobbyking/store/__27453__GWS_EP_Propeller_DD_5043_125x110mm_orange_6pcs_set_.html)

## 舵机连接

| 输出    | 频率    | 作动器   |
| ----- | ----- | ----- |
| MAIN1 | 50 Hz | 左副翼舵机 |
| MAIN2 | 50 Hz | 右副翼舵机 |
| MAIN3 | 50 Hz | 空     |
| MAIN4 | 50 Hz | 电机控制器 |