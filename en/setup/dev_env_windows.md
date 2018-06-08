# Windows Installation Instructions

There are a number of ways you can set up a Windows development toolchain for PX4. Depending on what option you choose these allow you to build for NuttX/Pixhawk and jMAVSim simulator targets.

> **Warning** The Windows toolchains are not officially supported (and we discourage their use). Depending on the option you choose they might be slow and not support all targets. Some options also cannot run the standard robotics software packages many developers use to prototype computer vision and navigation.

<span></span>
> **Tip** We recommend using [Ubuntu Linux](http://ubuntu.com) for development. Consider setting up a dual-boot setup next to your existing Windows installation.

| | [Cygwin Toolchain](../setup/dev_env_windows_cygwin.md) | [Virtual Machine Toolchain](../setup/dev_env_windows_vm.md) | [Bash on Windows Toolchain](../setup/dev_env_windows_bash_on_win.md) | [Msys Toolchain](../setup/dev_env_windows_msys.md) |
|---|---|---|---|---|
| Installation | MSI installer or Script | Manual (Hard) | Script | NSIS Installer |
| Native binary execution | yes | no | no | yes |
| Performance | ++ | -- | - | ++ |
| ARM Targets | ++ (quick) | + (VM USB) | + | - (broken) |
| Simulation jMAVSim | ++ | + | + | -- |
| Simulation gazebo | - (not yet) | + (slow) | + (slow) | -- |
| Support | + | ++ (Linux) | +/- | -- |
| Comments | <ul><li>New in 2018</li><li>Slim setup</li><li>Portable</li></ul> | <ul><li>Full Linux features</li><li>CPU & RAM intensive</li><li>Disk space intensive</li></ul> | <ul><li>Official Microsoft solution</li><li>Windows 10 only</li><li>Essentially a VM</li></ul> | <ul><li>No support</li><li>No documentation</li><li>No simulation</li></ul> |
