# Building PX4 Software

PX4 can be built on the console or in a graphical development environment / IDE.

## Compiling on the Console

Before moving on to a graphical editor or IDE, it is important to validate the system setup. Do so by bringing up the terminal. On OS X, hit ⌘-space and search for 'terminal'. On Ubuntu, click the launch bar and search for 'terminal'. On Windows, find the PX4 folder in the start menu and click on 'PX4 Console'.

![](images/toolchain/terminal.png)

The terminal starts in the home directory. We default to '~/src/Firmware' and clone the upstream repository. Experienced developers might clone [their fork](https://help.github.com/articles/fork-a-repo/) instead.

<div class="host-code"></div>

```sh
mkdir -p ~/src
cd ~/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --recursive
cd ..
```
Now its time to build the binaries by compiling the source code. But before going straight to the hardware, a [simulation run](simulation-sitl.md) is recommended as the next step. Users preferring to work in a graphical development environment should continue with the next section.

### NuttX / Pixhawk based boards

<div class="host-code"></div>

```sh
cd Firmware
make px4fmu-v2_default
```

Note the syntax: 'make' is the build tool, 'px4fmu-v2' is the hardware / autopilot version and 'default' is the default configuration. All PX4 build targets follow this logic. A successful run will end with this output:

<div class="host-code"></div>

```sh
[100%] Linking CXX executable firmware_nuttx
[100%] Built target firmware_nuttx
Scanning dependencies of target build_firmware_px4fmu-v2
[100%] Generating nuttx-px4fmu-v2-default.px4
[100%] Built target build_firmware_px4fmu-v2
```

By appending 'upload' to these commands the compiled binary will be uploaded via USB to the autopilot hardware:

<div class="host-code"></div>

```sh
make px4fmu-v2_default upload
```

A successful run will end with this output:

<div class="host-code"></div>

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```
### Raspberry Pi 2 boards
The command below builds the target for Raspbian (posix_pi2_release).

<div class="host-code"></div>

```sh
cd Firmware
make posix_rpi2_release # for cross-compiler build
```

The "mainapp" executable file is in the directory build_posix_rpi2_release/src/firmware/posix.
Copy it over to the RPi (replace YOUR_PI with the IP or hostname of your RPi, [instructions how to access your RPi](hardware-pi2.md#developer-quick-start))

```sh
scp build_posix_rpi2_release/src/firmware/posix/mainapp pi@YOUR_PI:/home/pi/
```

And run it with :
<div class="host-code"></div>

```sh
./mainapp
```


If you're building *directly* on the Pi, you will want the native build target (posix_pi2_default).

<div class="host-code"></div>

```sh
cd Firmware
make posix_rpi2_default # for native build
```

The "mainapp" executable file is in the directory build_posix_rpi2_default/src/firmware/posix.
Run it directly with :
<div class="host-code"></div>

```sh
./build_posix_rpi2_default/src/firmware/posix/mainapp
```

A successful build followed by executing mainapp will give you this :

```sh
[init] shell id: 1996021760
[init] task name: mainapp

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

Ready to fly.


pxh>
```

### QuRT / Snapdragon based boards

#### Build it

The commands below build the targets for the Linux and the DSP side. Both executables communicate via [muORB](advanced-uorb.md).

<div class="host-code"></div>

```sh
cd Firmware
make eagle_default
```

To load the SW on the device, connect via USB cable and make sure the device is booted. Run this in a new terminal window:

<div class="host-code"></div>

```sh
adb shell
```

Go back to previous terminal and upload:

<div class="host-code"></div>

```sh
make eagle_default upload
```

<aside class="note">
Note that this will also copy (and overwrite) the two config files [mainapp.config](https://github.com/PX4/Firmware/blob/master/posix-configs/eagle/flight/mainapp.config) and [px4.config](https://github.com/PX4/Firmware/blob/master/posix-configs/eagle/flight/px4.config) to the device. Those files are stored under /usr/share/data/adsp/px4.config and /home/linaro/mainapp.config respectively if you want to edit the startup scripts directly on your vehicle.
</aside>

The mixer currently needs to be copied manually:

<div class="host-code"></div>

```
adb push ROMFS/px4fmu_common/mixers/quad_x.main.mix  /usr/share/data/adsp
```

#### Run it

Run the DSP debug monitor:

<div class="host-code"></div>

```sh
${HEXAGON_SDK_ROOT}/tools/mini-dm/Linux_Debug/mini-dm
```

Go back to ADB shell and run mainapp:

```sh
cd /home/linaro
./mainapp mainapp.config
```

Note that the mainapp will stop as soon as you disconnect the USB cable (or if you ssh session is disconnected). To fly, you should make the mainapp auto-start after boot.

#### Auto-start mainapp

To run the mainapp as soon as the Snapdragon has booted, you can add the startup to `rc.local`:

Either edit the file `/etc/rc.local` directly on the Snapdragon:

```sh
adb shell
vim /etc/rc.local
```

Or copy the file to your computer, edit it locally, and copy it back:

```sh
adb pull /etc/rc.local
gedit rc.local
adb push rc.local /etc/rc.local
```

For the auto-start, add the following line before `exit 0`:

```
(cd /home/linaro && ./mainapp mainapp.config > mainapp.log)

exit 0
```

Make sure that the `rc.local` is executable:

```
adb shell
chmod +x /etc/rc.local
```

Then reboot the Snapdragon:

```
adb reboot
```

## Compiling in a graphical IDE

The PX4 system supports Qt Creator, Eclipse and Sublime Text. Qt Creator is the most user-friendly variant and hence the only officially supported IDE. Unless an expert in Eclipse or Sublime, their use is discouraged. Hardcore users can find an [Eclipse project](https://github.com/PX4/Firmware/blob/master/.project) and a [Sublime project](https://github.com/PX4/Firmware/blob/master/Firmware.sublime-project) in the source tree.

{% youtube %}https://www.youtube.com/watch?v=Bkk8zttWxEI&rel=0&vq=hd720{% endyoutube %}

## Qt Creator Functionality

Qt creator offers clickable symbols, auto-completion of the complete codebase and building and flashing firmware.

![](images/toolchain/qtcreator.png)

### Qt Creator on Linux

<aside class="note">
Linux users can just load the CMakeLists.txt in the root firmware folder via File -> Open File or Project -> Select the CMakeLists.txt file.
</aside>

After loading, the 'play' button can be configured to run the project by selecting 'custom executable' in the run target configuration and entering 'make' as executable and 'upload' as argument.

### Qt Creator on Windows

<aside class="todo">
Windows has not been tested with Qt creator yet.
</aside>

### Qt Creator on Mac OS

Before starting Qt Creator, the [project file](https://cmake.org/Wiki/CMake_Generator_Specific_Information#Code::Blocks_Generator) needs to be created:

<div class="host-code"></div>

```sh
cd ~/src/Firmware
mkdir build_creator
cd build_creator
cmake .. -G "CodeBlocks - Unix Makefiles"
```

That's it! Start Qt Creator, then complete the steps in the video below to set up the project to build.

{% youtube %}https://www.youtube.com/watch?v=0pa0gS30zNw&rel=0&vq=hd720{% endyoutube %}
