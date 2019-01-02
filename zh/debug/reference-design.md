# PX4 参考飞行控制器设计

PX4 参考设计是飞行控制器的 [Pixhawk 系列 ](https://docs.px4.io/en/flight_controller/pixhawk_series.html)。 该设计于2011年首次发布，现已进入第5 [代](#reference_design_generations)（第六代电路板设计正在进行中）。

## 二进制兼容性

所有按照特定设计制造的主板预计与二进制兼容（即可以运行相同的固件）。 从2018年起，我们将提供一个二进制兼容性测试套件，使我们能够验证兼容性。

FMU generations 1-3 were designed as open hardware, while FMU generations 4 and 5 provided only pinout and power supply specifications (schematics were created by individual manufacturers). In order to better ensure compatibility, FMUv6 and onward will return to a complete reference design model.

## Reference Design Generations {#reference_design_generations}

* FMUv1: Development board \(STM32F407, 128 KB RAM, 1MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv1)\) (no longer supported by PX4)
* FMUv2: Pixhawk \(STM32F427, 168 MHz, 192 KB RAM, 1MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv2)\)
* FMUv3: Pixhawk variants with 2MB flash \(3DR Pixhawk 2 \(Solo\), Hex Pixhawk 2.1, Holybro Pixfalcon, 3DR Pixhawk Mini, STM32F427, 168 MHz, 256 KB RAM, 2 MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv3_REV_D)\)
* FMUv4: Pixracer \(STM32F427, 168 MHz, 256 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1raRRouNsveQz8cj-EneWG6iW0dqGfRAifI91I2Sr5E0/edit#gid=1585075739)\) 
* FMUv4 PRO: Drotek Pixhawk 3 PRO \(STM32F469, 180 MHz, 384 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1raRRouNsveQz8cj-EneWG6iW0dqGfRAifI91I2Sr5E0/edit#gid=1585075739)\)
* FMUv5: Holybro Pixhawk 4 \(STM32F765, 216 MHz, 512 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165)\)
* FMUv6: work in progress, final name TBD, variant 6s \(STM32H7, 400 MHz, 2 MB RAM, 2 MB flash\) and variant 6i \(i.MX RT1050, 600 MHz, 512 KB RAM, external flash\)