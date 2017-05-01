---
translated_page: https://github.com/PX4/Devguide/blob/master/en/setup/dev_env_windows.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# Windows安装指南

> **警告：**虽然Windows上的工具链是可用的，但官方并不支持，我们不推荐使用Windows。Windows上固件编译的过程十分缓慢，且不支持新的板子，比如骁龙（Snapdragon Flight），它也不能运行标准机器人软件包，许多开发人员使用原型计算机视觉和导航。开始在Windows上开发之前，可以考虑安装一个双启动环境 [Ubuntu](http://www.ubuntu.com/index_kylin) 。

## 开发环境安装

下载并在系统上安装这些：

* [Qt Creator IDE](http://www.qt.io/download-open-source/#section-6)
* [PX4 Toolchain Installer v14 for Windows Download](http://firmware.diydrones.com/Tools/PX4-tools/px4_toolchain_installer_v14_win.exe) (32/64 bit systems, complete build system, drivers)
* [PX4 USB Drivers](http://pixhawk.org/static/px4driver.msi) (32/64 bit systems)
* [CMake](http://pan.baidu.com/s/1c1RgVgk)(这里推荐安装CMake-3.3.2-win32-x86)

现在继续运行： [代码编译](../setup/building_px4.md)!

## 新消息!Windows上的Bash
现在,那些想要在本地运行Bash shell进而按照Linux编译教程进行操作的Windows用户有了新的选择.请参见[BashOnWindows](https://github.com/Microsoft/BashOnWindows).我们已经进行了验证,PX4可以在此环境下成功编译.目前还无法用其刷写固件,但是你可以使用Mission Planner或者QGroundControl地面站刷固件.


