---
translated_page: https://github.com/PX4/Devguide/blob/master/en/sensor/sf1xx_lidar_setup.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# Lightware SF1XX 激光雷达设置
----------------------------------------------------


本页面向你展示如何去设置一个下面几种类型的激光雷达:

  1. SF10/a

  2. SF10/b

  3. SF10/c

  4. SF11/c

硬件驱动仅适合于飞控板上I2C的连接方式。


![](../../assets/hardware/sf1xx_i2c.jpg)


## 激光雷达设置
--------------------------------------------------------



首先通过USB连接传感器（内含USB转串口），运行终端，并按空格键检查I2C地址是否等于`0x66`。

新版本飞控板已经预先设置为`0x66`，而老版本设置的`0x55`地址与 `rgbled` 模块地址相冲突。



##  PX4设置
--------------------------------------------------------

通过 `SENS_EN_SF1XX` 参数选择激光雷达模型，然后重启。参数SENS_EN_SF1XX具体定义如下：

* `0` lidar disabled

* `1` SF10/a

* `2` SF10/b

* `3` SF10/c

* `4` SF11/c

