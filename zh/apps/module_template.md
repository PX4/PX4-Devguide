# 适用于开发完整应用的模版

PX4 固件中包含了一个模版文件： [src/templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module) ，基于该模版编写的应用（模块）可以在应用自己的栈上执行 [任务](../concept/architecture.md#runtime-environment) 。

> **Note** 所有在 [First Application Tutorial](../apps/hello_sky.md) 一节学到的内容都可用于编写完整的应用程序。

该模板主要展示了开发完整应用程序所需要的或者非常有用的如下附加功能：

- 访问参数并对参数更新做出反应。
- 订阅、等待 topic 更新。
- 通过 `start`/`stop`/`status` 控制后台运行的任务。 `module start [<arguments>]` 命令可以直接加入 [启动脚本](../concept/system_startup.md) 中。
- 命令行参数解析。
- 文档记录：`PRINT_MODULE_*` 方法有两个用处（该 API 在 [源代码](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381) 中有详细记录）： 
    - 它们可用于在控制台键入 `module help ` 指令后输出命令行指令的用法。
    - 可通过脚本提取该部分内容以自动生成 [Modules & Commands Reference](../middleware/modules_main.md) 页面。