# Parameters & Configurations

PX4 uses the *param subsystem* (a flat table of `float` and `int32_t` values) and text files (for mixers and startup scripts) to store its configuration.

This section discusses the *param* subsystem in detail.
It covers how to list, save and load parameters, and how to define them.

> **Note** [System startup](../concept/system_startup.md) and the way that [airframe configurations](../airframes/adding_a_new_frame.md) work are detailed on other pages. 


## Command Line Usage

The PX4 [system console](../debug/system_console.md) offers the [param](../middleware/modules_command.md#param) tool, which can be used to set parameters, read their value, save them, and export and restore to/from files.

### Getting and Setting Parameters

The `param show` command lists all system parameters:
```sh
param show
```

To be more selective, a partial parameter name with wildcard "*" can be used:
```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

You can use the `-c` flag to show all parameters that have changed (from their defaults):
```sh
param show -c
```

### Exporting and Loading Parameters

The standard `param save` command will store the parameters in the current default file:
```sh
param save
```

If provided with an argument, it will store the parameters instead to this new location:
```sh
param save /fs/microsd/vtol_param_backup
```

There are two different commands to load parameters: 
- `param load` replaces all the current parameters, duplicating the state when the parameters were stored.
   parameters are not automatically saved by this operation (it is assumed they are already saved).
- `param import` only updates parameter values that have been *changed from the default*.
  The parameters are automatically saved to the default location when loading.
  
  This can be used, for example, to just import calibration data without overwriting the rest of the system configuration (i.e. first calibrate a board that is otherwise unconfigured and save the parameters; this file can then be imported on another system to *just* update the calibration parameters).
  
Both loading options are shown below:
```sh
# Overwrite the current parameters
param load /fs/microsd/vtol_param_backup
param save  # Params not automatically saved on load

# Merge the current parameters with stored non-default parameters
param import /fs/microsd/vtol_param_backup  
```

## Parameter Names

Parameter names must be no more than 16 ASCII characters.

By convention, every parameter in a group should share the same (meaningful) string prefix followed by an underscore, and `MC_` and `FW_` are used for parameters related specifically to Multicopter or Fixed wing systems. This convention is not enforced.

The name must match in both code and [parameter metadata](#parameter_metadata) to correctly associate the parameter with its metadata (including default value in Firmware).


## C / C++ API

There are separate C and C++ APIs that can be used to access parameter values from within PX4 modules and drivers.

The most important difference between the APIs is that the C++ version has a more efficient standardized mechanism to synchronize with changes to parameter values (i.e. from a GCS).

Synchronization is important because a parameter can be changed to another value at any time.
Your code should *always* use the current value from the parameter store.
If getting the latest version is not possible, then a reboot will be required after the parameter is changed (set this requirement using the `@reboot_required` metadata).


### C++ API

The C++ API provides macros to declare parameters as *class attributes*. 
You add some "boilerplate" code to regularly listen for changes in the [uORB Topic](../middleware/uorb.md) associated with *any* parameter update.
Framework code then (invisibly) handles tracking uORB messages that affect your parameter attributes and keeping them in sync. 
In the rest of the code you can just use the defined parameter attributes and they will always be up to date!

First include **px4_module_params.h** in the class header for your module or driver (to get the `DEFINE_PARAMETERS` macro):
```cpp
#include <px4_module_params.h>
```

Derive your class from `ModuleParams`, and use `DEFINE_PARAMETERS` to specify a list of parameters and their associated parameter attributes.
The names of the parameters must be the same as their parameter metadata definitions.
```cpp
class MyModule : ..., public ModuleParams
{
public:
	...

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,   /**< example parameter */
		(ParamFloat<px4::params::ATT_BIAS_MAX>) _att_bias_max  /**< another parameter */
	)
};
```

Update the cpp file with boilerplate to check for the uORB message related to parameter updates.

First include the header to access the uORB parameter_update message:
```cpp
#include <uORB/topics/parameter_update.h>
```
Subscribe to the update message when the module/driver starts and un-subscribe when it is stopped. 
`parameter_update_sub` returned by `orb_subscribe()` is a handle we can use to refer to this particular subscription.
```cpp
# Subscribe to parameter_update message
int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
...
# Unsubscribe to parameter_update messages
orb_unsubscribe(parameter_update_sub);
```

Call `parameters_update(parameter_update_sub, true);` periodically in code to check if there has been an update (this is boilerplate):
```cpp
void Module::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	// Check if any parameter updated
	orb_check(parameter_update_sub, &updated);

	// If any parameter updated copy it to: param_upd
	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	// If any parameter updated, call updateParams() to check if
    // this class attributes need updating (and do so). 
	if (force || updated) {
		updateParams();
	}
}
```
In the above method:
- `orb_check()` tells us if there is *any* update to the `param_update` uORB message (but not what parameter is affected) and sets the `updated` bool.
- If there has been "some" parameter updated, we then copy the update into a `parameter_update_s` (`param_upd`)
- Last of all we call `ModuleParams::updateParams()`.
  This "under the hood" checks if the specific parameter attributes listed in our `DEFINE_PARAMETERS` list need updating, and then does so if needed.

The parameter attributes (`_sys_autostart` and `_att_bias_max` in this case) can then be used to represent the parameters, and will be updated whenever the parameter value changes.

> **Tip** The [Application/Module Template](../apps/module_template.md) uses the new-style C++ API but does not include [parameter metadata](#parameter_metadata).


### C API

The C API can be used within both modules and drivers.

First include the parameter API:
```C
#include <parameters/param.h>
```

Then retrieve the parameter and assign it to a variable (here `my_param`), as shown below for `PARAM_NAME`.
The variable `my_param` can then be used in your module code.
```C
int32_t my_param = 0;
param_get(param_find("PARAM_NAME"), &my_param);
```

> **Note** If `PARAM_NAME` was declared in parameter metadata then its default value will be set, and the above call to find the parameter should always succeed. 

`param_find()` is an "expensive" operation, which returns a handle that can be used by `param_get()`. 
If you're going to read the parameter multiple times, you may cache the handle and use it in `param_get()` when needed
```cpp
# Get the handle to the parameter
int32_t my_param_handle = 0;
my_param_handle = param_find("PARAM_NAME");

# Query the value of the parameter when needed
int32_t my_param = 0;
param_get(my_param_handle, &my_param);
```


## Parameter Meta Data {#parameter_metadata}

PX4 uses an extensive parameter metadata system to drive the user-facing presentation of parameters, and to set the default value for each parameter in firmware.

> **Tip** Correct meta data is critical for good user experience in a ground station.

Parameter metadata can be stored anywhere in the source tree, in a file with extension **.c**.
Typically it is stored alongside its associated module. 

The build system extracts the metadata (using `make parameters_metadata`) to build the [parameter reference](../advanced/parameter_reference.md) and the parameter information used by ground stations.

Parameter metadata sections look like the following examples:

```cpp
/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.0005
 * @reboot_required true
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```
```cpp
/**
 * Acceleration compensation based on GPS
 * velocity.
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_ACC_COMP, 1);
```

The `PARAM_DEFINE_*` macro at the end specifies the type of parameter (`PARAM_DEFINE_FLOAT` or `PARAM_DEFINE_INT32`), the name of the parameter (which must match the name used in code), and the default value in firmware.

The lines in the comment block are all optional, and are primarily used to control display and editing options within a ground station.
The purpose of each line is given below (for more detail see [module_schema.yaml](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml)).

```cpp
/**
 * <title>
 *
 * <longer description, can be multi-line>
 *
 * @unit <the unit, e.g. m for meters>
 * @min <the minimum sane value. Can be overridden by the user>
 * @max <the maximum sane value. Can be overridden by the user>
 * @decimal <the minimum sane value. Can be overridden by the user>
 * @increment <the "ticks" in which this value will increment in the UI>
 * @reboot_required true <add this if changing the param requires a system restart.>
 * @boolean <add this for integer parameters that represent a boolean value>
 * @group <a title for parameters that form a group>
 */
```
