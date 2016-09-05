# System Startup

The PX4 boot is controlled by shell scripts in the [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) folder.

All files starting with a number and underscore (e.g. `10000_airplane`) are canned airframe configurations. They are exported at build-time into an `airframes.xml` file which is parsed by [QGroundControl](http://qgroundcontrol.com) for the airframe selection UI. Adding a new configuration is covered [here](airframes-adding-a-new-frame.md).

The remaining files are part of the general startup logic, and the first executed file is the [rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rcS) script, which calls all other scripts.

## Debugging the System Boot

A failure of a driver of software component can lead to an aborted boot.

<aside class="tip">
An incomplete boot often materializes as missing parameters in the ground control stations, because the non-started applications did not initialize their parameters.
</aside>

The right approach to debug the boot sequence is to connect the [system console](advanced-system-console.md) and power-cycle the board. The resulting boot log has detailed information about the boot sequence and should contain hints why the boot aborted.

### Common boot failure causes

  * A required sensor failed to start
  * For custom applications: The system was out of RAM. Run the `free` command to see the amount of free RAM.
  * A software fault or assertion resulting in a stack trace

## Replacing the System Startup

In most cases customizing the default boot is the better approach, which is documented below. If the complete boot should be replaced, create a file `/fs/microsd/etc/rc.txt`, which is located in the `etc` folder on the microSD card. If this file is present nothing in the system will be auto-started.

## Customizing the System Startup

The best way to customize the system startup is to introduce a [new airframe configuration](airframes-adding-a-new-frame.md). If only tweaks are wanted (like starting one more application or just using a different mixer) special hooks in the startup can be used.

<aside class="caution">
The system boot files are UNIX FILES which require UNIX LINE ENDINGS. If editing on Windows use a suitable editor.
</aside>

There are three main hooks. Note that the root folder of the microsd card is identified by the path `/fs/microsd`.

  * /fs/microsd/etc/config.txt
  * /fs/microsd/etc/extras.txt
  * /fs/microsd/mixers/NAME_OF_MIXER

### Customizing the Configuration (config.txt)

The `config.txt` file is loaded after the main system has been configured and *before* it is booted and allows to modify shell variables.

### Starting additional applications

The `extras.txt` can be used to start additional applications after the main system boot. Typically these would be payload controllers or similar optional custom components.

### Starting a custom mixer

By default the system loads the mixer from `/etc/mixers`. If a file with the same name exists in `/fs/microsd/etc/mixers` this file will be loaded instead. This allows to customize the mixer file without the need to recompile the Firmware.
