# 适用于开发完整应用的模版

PX4 固件中包含了一个模版文件： [src/templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module) ，基于该模版编写的应用（模块）可以在应用自己的栈上执行 [任务](../concept/architecture.md#runtime-environment) 。

> **Note** 所有在 [First Application Tutorial](../apps/hello_sky.md) 一节学到的内容都可用于编写完整的应用程序。

该模板主要展示了开发完整应用程序所需要的或者非常有用的如下附加功能：

- 访问参数并对参数更新做出反应。
- 订阅、等待 topic 更新。
- 通过 `start`/`stop`/`status` 控制后台运行的任务。 The `module start [<arguments>]` command can then be directly added to the [startup script](../concept/system_startup.md).
- Command-line argument parsing.
- Documentation: the `PRINT_MODULE_*` methods serve two purposes (the API is documented [in the source code](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381)): 
    - They are used to print the command-line usage when entering `module help` on the console.
    - They are automatically extracted via script to generate the [Modules & Commands Reference](../middleware/modules_main.md) page.