# Module Template for Full Applications

The PX4 Firmware contains a template for writing a new application (module) that runs as a [task](../concept/architecture.md#runtime-environment) on its own stack:
[src/templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module).

> **Note** All the things learned in the [First Application Tutorial](../apps/hello_sky.md) are relevant for writing a full application.

The template demonstrates the following additional features/aspects that are required or are useful for a full application:

- Accessing parameters and reacting to parameter updates.
- uORB subscriptions and waiting for topic updates.
- Controlling the task that runs in the background via `start`/`stop`/`status`.
  The `module start [<arguments>]` command can then be directly added to the
  [startup script](../concept/system_startup.md).
- Command-line argument parsing.
- Documentation: the `PRINT_MODULE_*` methods serve two purposes (the API is
  documented [in the source code](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381)):
  - They are used to print the command-line usage when entering `module help` on the console.
  - They are automatically extracted via script to generate the [Modules & Commands Reference](../middleware/modules_main.md) page.

