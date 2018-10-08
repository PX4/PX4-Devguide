# Adding a Serial Port Driver

This tutorial documents the necessary steps to add a serial driver that can be set to run on any of the configurable ports a board provides.

It assumes that you already have the driver and you can start it on shell via `<driver> start -d <serial_port>`.

If the driver supports several baudrates, it should also be accepted via command-line argument (e.g. `-b <baudrate>`).
The baudrate must also be accepted as a parameter name in the form of `-b p:<param_name>` and can be parsed with `px4_get_parameter_value()`.
Have a look at the [gps driver](https://github.com/PX4/Firmware/blob/master/src/drivers/gps/gps.cpp#L1023) for an example.

Making the driver configurable consists of 2 steps:
1. creating a YAML module configuration file. Add a new file in the driver's source directory named `module.yaml`, insert the following and adjust as needed:
 ```
module_name: uLanding Radar
serial_config:
    - command: ulanding_radar start -d ${SERIAL_DEV} -b p:${BAUD_PARAM}
      port_config_param:
        name: SENS_ULAND_CFG
        group: Sensors
 ```
  The full documentation of the module configuration file can be found in the [validation/module_schema.yaml](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml) file.
  This is also used to validate all configuration files in CI.
1. adding the module configuration to the CMakeLists.txt file, like:
  ```
px4_add_module(
	MODULE drivers__ulanding
	MAIN ulanding_radar
	SRCS
		ulanding.cpp
	MODULE_CONFIG
		module.yaml
	)
  ```



