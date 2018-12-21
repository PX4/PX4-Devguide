# Windows 安装指南

如果希望在Windows平台进行PX4的开发，请参考： [Windows Cygwin 工具链](../setup/dev_env_windows_cygwin.md)进行工具链的安装。

> **Tip** The *Cygwin 工具链* 仅支持NuttX/Pixhawk 平台和 jMAVSim仿真平台。 如果你想编译用于 [其他硬件平台](/setup/dev_env.md#supported-targets)的代码，可以考虑额外安装一个 [Ubuntu Linux](http://ubuntu.com)组成双系统环境。

## 额外工具

完成编译/仿真开发环境设置后，你可以从 [额外工具](../setup/generic_dev_tools.md) 找到一些有用的“通用”开发工具。

## 后续步骤

设置完环境后，请转至 构建说明</0 > 进行编译测试。</p> 

## 其他 windows 工具链

除 Cygwin 外开发者们还可以使用一些替代解决方案完成开发环境的构建， 下表对这些替代解决方案进行了详细的对比。

> **注意** Windows平台下仅 [Cygwin 工具链](../setup/dev_env_windows_cygwin.md) 获得了PX4开发团队的官方支持。 它作为我们持续集成系统的一部分会被定期测试，在性能方面要比其它替代方案更出色。

|                         | [Cygwin 工具链](../setup/dev_env_windows_cygwin.md) **(官方支持)** | [虚拟机工具链](../setup/dev_env_windows_vm.md) | [Bash on Windows 工具链](../setup/dev_env_windows_bash_on_win.md) | [Msys 工具链](../setup/dev_env_windows_msys.md) |
| ----------------------- | ----------------------------------------------------------- | ---------------------------------------- | -------------------------------------------------------------- | -------------------------------------------- |
| 安装方式                    | MSI安装包/脚本                                                   | 手动安装 (硬核玩家)                              | 脚本                                                             | NSIS安装包                                      |
| Native binary execution | 是                                                           | 否                                        | 否                                                              | 是                                            |
| 性能                      | ++                                                          | --                                       | -                                                              | ++                                           |
| ARM平台                   | ++ (速度快)                                                    | + (VM USB)                               | +                                                              | - (功能损坏)                                     |
| 仿真 - jMAVSim            | ++                                                          | +                                        | +                                                              | --                                           |
| 仿真 - gazebo             | - (暂不支持)                                                    | + (速度稍慢)                                 | + (速度稍慢)                                                       | --                                           |
| 技术支持                    | +                                                           | ++ (Linux)                               | +/-                                                            | --                                           |
| 备注:                     |                                                             |                                          |                                                                |                                              |

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