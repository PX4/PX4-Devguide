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
- Execute `make posix EXTERNAL_MODULES_LOCATION=<path>`. Any other build target
  can be used, but the build directory must not yet exist. If it already exists,
  you can also just set the cmake variable in the build folder.
  For the following incremental builds `EXTERNAL_MODULES_LOCATION` does not need
  to be specified anymore.

