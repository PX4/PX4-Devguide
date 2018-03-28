# Module Template to write a Full Application

The PX4 Firmware contains a template for writing a new application, which can be
found under
[src/templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module).
It can be used for writing an application that runs on its own
stack (as opposed to a work-queue task. This is explained in more detail on
the [Architecture Overview page](../concept/architecture.md#runtime-environment)).

> **Note** All the things learned in the [First Application
> Tutorial](tutorial_hello_sky.md) are relevant for writing a full application,
> but the template contains a few more things.

The template includes various aspects that are required or are useful for a full
application:

- Accessing parameters and reacting to parameter updates.
- uORB subscriptions and waiting for topic updates.
- Controlling the task that runs in the background via `start`/`stop`/`status`.
  The `module start [<arguments>]` command can then be directly added to the
  [startup script](../advanced/system_startup.md).
- Command-line argument parsing.
- Documentation: the `PRINT_MODULE_*` methods serve two purposes (and the API is
  documented [in the source code](https://github.com/PX4/Firmware/blob/master/src/platforms/px4_module.h#L387)):
  - They are used to print the command-line usage when entering `module help` on the console.
  - They are automatically extracted via script to generate the
[Modules & Commands Reference](../middleware/modules_main.md) page.


