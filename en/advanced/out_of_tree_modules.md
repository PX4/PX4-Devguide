# Out-of-tree Modules

This describes the possibility to add an external module to the PX4 build.

External modules can use the same includes as internal modules and can interact
with internal modules via uORB.

## Usage

- `EXTERNAL_MODULES_LOCATION` needs to point to a directory with the same
  structure as Firmware (and thus contains a directory called `src`).
- There are two options: copy an existing module (eg. examples/px4_simple_app)
  to the external directory, or directly create a new module.
- Rename the module (including `MODULE` in CMakeLists.txt) or remove it from the
  existing Firmware cmake build config. This is to avoid conflicts with internal
  modules.
- Add a file `$EXTERNAL_MODULES_LOCATION/CMakeLists.txt` with content:

```
set(config_module_list_external
    modules/<new_module>
    PARENT_SCOPE
    )
```
- add a line `EXTERNAL` to the `modules/<new_module>/CMakeLists.txt` within
  `px4_add_module`, for example like this:

```
px4_add_module(
	MODULE modules__test_app
	MAIN test_app
	STACK_MAIN 2000
	SRCS
		px4_simple_app.c
	DEPENDS
		platforms__common
	EXTERNAL
	)

```


# Out-of-tree uORB Message Definitions

uORB messages can also be defined out-of-tree. For this, the
`$EXTERNAL_MODULES_LOCATION/msg` needs to exist.

- Place all new message definitions within the `$EXTERNAL_MODULES_LOCATION/msg`
  directory. The format of these new out-of-tree message definitions are the same
  as for any other [uORB message definition](../middleware/uorb.md#adding-a-new-topic).
- Add a file `$EXTERNAL_MODULES_LOCATION/msg/CMakeLists.txt` with content:

```
set(config_msg_list_external
    <message1>.msg
    <message2>.msg
    <message3>.msg
    PARENT_SCOPE
    )

```
where `<message#>.msg` is the name of the uORB message definition file to be processed 
and used for uORB message generation.

The out-of-tree uORB messages will be generated in the same locations as the normal uORB messages.
The uORB topic headers are generated in `<build_dir>/uORB/topics/`, and the message source files are
generated in `<build_dir>/msg/topics_sources/`.

The new uORB messages can be used like any other uORB message as described [here](../middleware/uorb.md#adding-a-new-topic).

> **Warning** The out-of-tree uORB message definitions cannot have the same name as any of the normal uORB
messages.

# Building Out-of-tree modules and uORB Messages

- Execute `make posix EXTERNAL_MODULES_LOCATION=<path>`. Any other build target
  can be used, but the build directory must not yet exist. If it already exists,
  you can also just set the cmake variable in the build folder.
  For the following incremental builds `EXTERNAL_MODULES_LOCATION` does not need
  to be specified anymore.
