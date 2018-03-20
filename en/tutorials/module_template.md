# Module Template

The PX4 Firmware contains a template for writing a new application, which can be
found under
[src/templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module).
It can be used for writing an application that runs on its own
stack (as opposed to a work-queue task. This will be explained in more detail on
the [Architecture Overview page](../concept/architecture.md#runtime-environment)).

The template includes various aspects that might be useful for a full
application:

- Accessing parameters and reacting to parameter updates.
- uORB subscriptions and waiting for topic updates.
- Controlling the task that runs in the background via `start`/`stop`/`status`.
  The `module start [<arguments>]` command can then be directly added to the
  startup script.
- Command-line argument parsing.
- Documentation: the `PRINT_MODULE_*` methods serve two purposes:
  - They are used to print the command-line usage when entering `module help` on the console.
  - They are automatically extracted via script to generate the
[Modules & Commands Reference](../middleware/modules_main.md) page.


