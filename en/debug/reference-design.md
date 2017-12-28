# Reference Design

The reference design for PX4 is since 2011 the Pixhawk series of flight controllers. Designed by the same team, it is now in its 5th generation and the 6th generation design is under way.

In the past no formal binary compatibility program was in place, however, starting 2018 we will offer a test suite to ensure board manufactured to the standard are indeed binary compatible. FMU generations 1-3 were designed as open hardware, while FMU generations 4 and 5 were only pinout and power supply specifications. Schematics were done by individual manufacturers. FMUv6 and onward will return to a complete reference design model to ensure consistency.

## Reference Design Generations

* FMUv1: Development board \(STM32F407, 128 KB RAM, 1MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv1)\)
* FMUv2: Pixhawk \(STM32F427, 168 MHz, 192 KB RAM, 1MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv2)\)
* FMUv3: Pixhawk variants with 2MB flash \(3DR Pixhawk 2 \(Solo\), Hex Pixhawk 2.1, Holybro Pixfalcon, 3DR Pixhawk Mini, STM32F427, 168 MHz, 192 KB RAM, 2 MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv2)\)
* FMUv4: Pixracer \(STM32F427, 168 MHz, 192 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1raRRouNsveQz8cj-EneWG6iW0dqGfRAifI91I2Sr5E0/edit#gid=1585075739)\)
* FMUv4 PRO: Drotek Pixhawk 3 PRO \(STM32F467, 168 MHz, 384 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1raRRouNsveQz8cj-EneWG6iW0dqGfRAifI91I2Sr5E0/edit#gid=1585075739)\)
* FMUv5: final name TBD \(STM32F7675, 200 MHz, 512 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165)\)
* FMUv6: work in progress, final name TBD, variant 6s \(STM32H7, 400 MHz, 2 MB RAM,  2 MB flash\) and variant 6i \(i.MX RT1050, 600 MHz, 512 KB RAM, external flash\)



