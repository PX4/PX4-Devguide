# 模块参考：驱动
## fmu
源代码: [drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)


### 说明
该模块负责驱动输出以及读取输入引脚。对于没有单独IO芯片的飞控板（例如Pixracer），它使用主通道。在具有IO芯片（例如Pixhawk）的飞控板上，它使用辅助通道，px4io驱动程序使用主通道。

它通过监听actuator_controls主题，实现混控以及PWM输出。此外，它还进行RC输入解析并自动选择解析方法。支持的方法有：
- PPM
- SBUS
- DSM
- SUMD
- ST24

该模块通过mode_ *命令配置。 这定义了驱动程序应该占用的前N个引脚。例如，通过使用mode_pwm4，引脚5和6可以用于相机触发驱动或PWM测距仪驱动。

### 实现
默认情况下，模块在工作队列上运行，以减少RAM的使用。 它也可以在独占线程中运行，通过标志-t启用，以减少延迟。

当在工作队列上运行时，它以固定的频率被调用，pwm速率限制actuator_controls主题的更新速率。在独占线程中运行的情况下，模块会轮询actuator_controls主题。此外，pwm速率定义了较低级别的IO定时器速率。

### 示例
通常以以下方式启动：
```
fmu mode_pwm
```
驱动所有可用引脚。

捕获输入（上升沿和下降沿）并在控制台上打印：以其中一种捕获模式启动fmu：
```
fmu mode_pwm3cap1
```
这将能够在第4个引脚上捕获。 然后：
```
fmu test
```

使用`pwm`命令进一步配置（PWM速率，电平，...），使用`mixer`命令加载混控器文件。

### 用法
```
fmu <命令> [参数...]
 命令:
   start         启动任务（没有任何模式设置，使用任意mode_ *均可）
     [-t]        运行在独占线程而不是工作队列上

 如果fmu不在运行状态，那么所有的mode_*命令都会启动它

   mode_gpio

   mode_rcin     只做RC输入，不做PWM输出

   mode_pwm      选择所有可用引脚作为PWM输出

   mode_pwm1

   mode_pwm4

   mode_pwm2

   mode_pwm3

   mode_pwm3cap1

   mode_pwm2cap2

   mode_serial

   mode_gpio_serial

   mode_pwm_serial

   mode_pwm_gpio

   bind          发送DSM绑定命令（模块必须正在运行）

   sensor_reset  执行传感器重置(SPI bus)
     [<ms>]      在重置和重启之间的延迟时间，单位：ms

   peripheral_reset 重置所有外设
     [<ms>]      在重置和重启之间的延迟时间，单位：ms

   i2c           配置I2C时钟速率
     <bus_id> <rate> 指定总线id(>=0)和速率，单位：Hz

   test          测试输入输出

   fake          解锁并发送作动器控制指令
     <roll> <pitch> <yaw> <thrust> 控制量，取值范围：[-100, 100]

   stop

   status        打印状态信息
```
