# Windows 安装指南

如果希望在Windows平台进行PX4的开发，请参考： [Windows Cygwin Toolchain](../setup/dev_env_windows_cygwin.md)进行工具链的安装。

> **Tip** The *Cygwin 工具链* 仅支持NuttX/Pixhawk 平台和 jMAVSim仿真平台。 如果你想编译用于 [其他硬件平台](/setup/dev_env.md#supported-targets)的代码，可以考虑额外安装一个 [Ubuntu Linux](http://ubuntu.com)组成双系统环境。

## 额外工具

完成编译/仿真开发环境设置后，你可以从 [额外工具](../setup/generic_dev_tools.md) 找到一些有用的“通用”开发工具。

## 后续步骤

设置完环境后，请转至 构建说明</0 > 进行编译测试。</p> 

## 其他 windows 工具链

除 Cygwin 外开发者们还可以使用一些替代解决方案完成开发环境的构建， 下表对这些替代解决方案进行了详细的对比。

> **Note** The [Cygwin Toolchain](../setup/dev_env_windows_cygwin.md) is the only one that is supported by the PX4 dev team. It is regularly tested as part of our continuous integration system and is known to be better performing than the other alternatives.

|                         | [Cygwin Toolchain](../setup/dev_env_windows_cygwin.md) **(Supported)** | [Virtual Machine Toolchain](../setup/dev_env_windows_vm.md) | [Bash on Windows Toolchain](../setup/dev_env_windows_bash_on_win.md) | [Msys Toolchain](../setup/dev_env_windows_msys.md) |
| ----------------------- | ---------------------------------------------------------------------- | ----------------------------------------------------------- | -------------------------------------------------------------------- | -------------------------------------------------- |
| Installation            | MSI installer or Script                                                | Manual (Hard)                                               | Script                                                               | NSIS Installer                                     |
| Native binary execution | yes                                                                    | no                                                          | no                                                                   | yes                                                |
| Performance             | ++                                                                     | --                                                          | -                                                                    | ++                                                 |
| ARM Targets             | ++ (quick)                                                             | + (VM USB)                                                  | +                                                                    | - (broken)                                         |
| Simulation jMAVSim      | ++                                                                     | +                                                           | +                                                                    | --                                                 |
| Simulation gazebo       | - (not yet)                                                            | + (slow)                                                    | + (slow)                                                             | --                                                 |
| Support                 | +                                                                      | ++ (Linux)                                                  | +/-                                                                  | --                                                 |
| Comments                |                                                                        |                                                             |                                                                      |                                                    |

- New in 2018
- Slim setup
- Portable

|

- Full Linux features
- CPU & RAM intensive
- Disk space intensive

|

- Simulation UI is a "hack".
- Windows 10 only
- Essentially a VM

|

- No support
- No documentation
- No simulation

|