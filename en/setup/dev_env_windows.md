# Windows Installation Instructions

There are a number of ways you can set up a Windows development toolchain for PX4. Depending on what option you choose these allow you to build for NuttX/Pixhawk and jMAVSim simulator targets.

> **Warning** The Windows toolchains are not officially supported (and we discourage their use). Depending on the option you choose they might be slow and not support all targets. Some options also cannot run the standard robotics software packages many developers use to prototype computer vision and navigation.

<span></span>
> **Tip** To work on an unrestriced linux environment you can install [Ubuntu](http://ubuntu.com) in a dual-boot setup next to your existing Windows installation and choose the operating system on every bootup.

## Cygwin based Toolchain

[Cygwin Toolchain Download & Instructions](../setup/dev_env_windows_msys.md)

**\+**
+ Fast and simple setup via MSI installer
+ Supports all basic functionality (see instructions)
+ No virtual machine and additional operating system overhead
+ Native Windows binary execution

**\-**
- Does not support all features of the PX4 environment
- Known issues see link to issue trakcing on the instructions page

## Virtual Machine-Hosted Toolchain

[VM Toolchain Instructions](../setup/dev_env_windows_vm.md)

**\+**
+ Support for all the features of the PX4 environment
+ Exact same environment like descibed in all linux instructions

**\-**
- Huge disk space, RAM and CPU overhead
- Performance issues with graphics

## Bash on Windows based Toolchain

[Bash on Windows Toolchain](../setup/dev_env_windows_bash_on_win.md)

**\+**
+ Official POSIX environment solution offered by Microsoft

**\-**
- Similar to VM just with less memory overhead
- Doesn't work on Windows 7, 8 (only 10)

## Msys based Toolchain

[Msys Toolchain](../setup/dev_env_windows_msys.md)

**\+**
+ Native Windows binary execution
+ No virtual machine and additional operating system overhead

**\-**
- Available package years old and currently unsupported (**builds failing**)
- No detailed documentation
- Cannot run simulation, only NuttX builds
