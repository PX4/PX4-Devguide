# Parameters & Configurations

The PX4 platform uses the param subsystem (a flat table of float and int32_t values) and text files (for mixers and startup scripts) to store its configuration.

The [system startup](advanced-system-startup.md) and how [airframe configurations](airframes-adding-a-new-frame.md) work are detailed on other pages. This section discusses the param subsystem in detail

## Command Line usage

The PX4 [system console](advanced-system-console.md) offers the ```param``` tool, which allows to set parameters, read their value, save them and export and restore to and from files.

### Getting and Setting Parameters

The param show command lists all system parameters:

```sh
param show
```

To be more selective a partial parameter name with wildcard can be used:

```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

### Exporting and Loading Parameters

The standard save command will store the parameters in the current default file:

```sh
param save
```

If provided with an argument, it will store the parameters instead to this new location:

```sh
param save /fs/microsd/vtol_param_backup
```

There are two different commands to load parameters: ```param load``` will load a file and replace the current parameters with what is in this file, resulting in a 1:1 copy of the state that was present when these parameters were stored. ```param import``` is more subtle: It will only change parameter values which have been changed from the default. This is great to e.g. initially calibrate a board (without configuring it further), then importing the calibration data without overwriting the rest of the system configuration.

Overwrite the current parameters:

```sh
param load /fs/microsd/vtol_param_backup
```

Merge the current parameters with stored parameters (stored values which are non-default take precedence):

```sh
param import /fs/microsd/vtol_param_backup
```

## C / C++ API

There is also a C and a separate C++ API which can be used to access parameter values.

<aside class="todo">
Discuss param C / C++ API.
</aside>

<div class="host-code"></div>

```C
int32_t param = 0;
param_get(param_find("PARAM_NAME"), &param);
```

## Parameter Meta Data

PX4 uses an extensive parameter meta data system to drive the user-facing presentation of parameters. Correct meta data is critical for good user experience in the ground station.

A typical parameter metadata section will look like this:

```C++
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

Where each line has this use:

```C++
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
 * @reboot_required true <add this if changing the param requires a system restart>
 * @group <a title for parameters which form a group>
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```

