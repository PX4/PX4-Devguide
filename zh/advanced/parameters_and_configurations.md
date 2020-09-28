# 参数 & 配置

PX4 使用 *param subsystem *（`float` 和 `int32_t` 值的平面表）和文本文件（用于混频器和启动脚本）来存储其配置。

本节详细讨论 *param* 子系统。 它包括如何列出、保存和加载参数，以及如何定义它们。

> **Note** 在其他页面上详细介绍了 [System 启动 ](../concept/system_startup.md) 和 [airframe 配置 ](../airframes/adding_a_new_frame.md) 工作方式。

## 命令行使用方法

PX4 system 控制台/0 > 提供了 [param](../middleware/modules_command.md#param) 工具，可用于设置参数、读取其值、保存参数以及从文件中导出和还原参数。</p> 

### 获取和设置参数

`param show ` 命令列出了所有系统参数:

```sh
param show
```

为了更有选择性，可以使用带有通配符 "*" 的部分参数名称：

```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

可以使用 `-c` 标志显示已更改的所有参数（从其默认值）：

```sh
param show -c
```

You can use `param show-for-airframe` to show all parameters that have changed from their defaults for just the current airframe's definition file (and defaults it imports).

### 导出和加载参数

You can save any parameters that have been *touched* since all parameters were last reset to their firmware-defined defaults (this includes any parameters that have been changed, even if they have been changed back to their default).

The standard `param save` command will store the parameters in the current default file:

```sh
param save
```

If provided with an argument, it will store the parameters instead to this new location:

```sh
param save /fs/microsd/vtol_param_backup
```

There are two different commands to *load* parameters:

- `param load ` 首先将所有参数完全重置为默认值，然后用存储在文件中的任何值覆盖参数值。
- `param import ` 只是用文件中的值覆盖参数值，然后保存结果（即有效调用 `param save</0 >）。</li>
</ul>

<p>The <code>load` effectively resets the parameters to the state when the parameters were saved (we say "effectively" because any parameters saved in the file will be updated, but other parameters may have different firmware-defined default values than when the parameters file was created).</p> 
    By contrast, `import` merges the parameters in the file with the current state of the vehicle. This can be used, for example, to just import a parameter file containing calibration data, without overwriting the rest of the system configuration.
    
    Examples for both cases are shown below:
    
    ```sh
    # 将参数重置为保存文件时,
    param load /fs/microsd/vtol_param_backup
    # 保存参数 (不自动完成与负载)
    param save
    ```
    
    ```sh
    # 将保存的参数与当前参数合并
    param import /fs/microsd/vtol_param_backup  
    ```
    
    ## 参数名称
    
    Parameter names must be no more than 16 ASCII characters.
    
    By convention, every parameter in a group should share the same (meaningful) string prefix followed by an underscore, and `MC_` and `FW_` are used for parameters related specifically to Multicopter or Fixed wing systems. This convention is not enforced.
    
    The name must match in both code and [parameter metadata](#parameter_metadata) to correctly associate the parameter with its metadata (including default value in Firmware).
    
    ## C / C++ API
    
    There are separate C and C++ APIs that can be used to access parameter values from within PX4 modules and drivers.
    
    One important difference between the APIs is that the C++ version has a more efficient standardized mechanism to synchronize with changes to parameter values (i.e. from a GCS).
    
    Synchronization is important because a parameter can be changed to another value at any time. Your code should *always* use the current value from the parameter store. If getting the latest version is not possible, then a reboot will be required after the parameter is changed (set this requirement using the `@reboot_required` metadata).
    
    In addition, the C++ version has also better type-safety and less overhead in terms of RAM. The drawback is that the parameter name must be known at compile-time, while the C API can take a dynamically created name as a string.
    
    ### C++ API
    
    The C++ API provides macros to declare parameters as *class attributes*. You add some "boilerplate" code to regularly listen for changes in the [uORB Topic](../middleware/uorb.md) associated with *any* parameter update. Framework code then (invisibly) handles tracking uORB messages that affect your parameter attributes and keeping them in sync. In the rest of the code you can just use the defined parameter attributes and they will always be up to date!
    
    First include **px4_platform_common/module_params.h** in the class header for your module or driver (to get the `DEFINE_PARAMETERS` macro):
    
    ```cpp
    #include <px4_platform_common/module_params.h>
    ```
    
    Derive your class from `ModuleParams`, and use `DEFINE_PARAMETERS` to specify a list of parameters and their associated parameter attributes. The names of the parameters must be the same as their parameter metadata definitions.
    
    ```cpp
    class MyModule : ..., public ModuleParams
    {
    public:
        ...
    
    private:
    
        /**
         * Check for parameter changes and update them if needed.
         * @param parameter_update_sub uorb subscription to parameter_update
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
    
    Subscribe to the update message when the module/driver starts and un-subscribe when it is stopped. `parameter_update_sub` returned by `orb_subscribe()` is a handle we can use to refer to this particular subscription.
    
    ```cpp
    # Subscribe to parameter_update message
    int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    ...
    # Unsubscribe to parameter_update messages
    orb_unsubscribe(parameter_update_sub);
    ```
    
    Call `parameters_update(parameter_update_sub);` periodically in code to check if there has been an update (this is boilerplate):
    
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
    
        if (force || updated) {
            // If any parameter updated, call updateParams() to check if
            // this class attributes need updating (and do so). 
            updateParams();
        }
    }
    ```
    
    In the above method:
    
    - `orb_check()` 告诉我们是否有 *任何* 更新 `param_update` 的 uorb 消息 (但不是受影响的参数)，并设置 `updated` bool。
    - 如果更新了 "某些" 参数，我们会将更新复制到 `parameter_update_s` (`param_upd`)
    - 调用 `ModuleParams::updateParams()`。 在此检查 `DEFINE_PARAMETERS` 列表中列出的特定参数属性是否需要更新，然后在需要时进行更新。
    - 当 `force=True` 时，此示例不调用 `Module::p arameters_update()`。 如果您有其他需要设置公共模式的值，则是将它们包含在函数中，并在初始化过程中使用 `force=True` 调用它一次。
    
    The parameter attributes (`_sys_autostart` and `_att_bias_max` in this case) can then be used to represent the parameters, and will be updated whenever the parameter value changes.
    
    > **Tip** [Application/Module templateet](../apps/module_template.md) 使用新型 C++ API，但不包括 [parameter 元目录 ](#parameter_metadata)。
    
    ### C API
    
    The C API can be used within both modules and drivers.
    
    First include the parameter API:
    
    ```C
    #include <parameters/param.h>
    ```
    
    Then retrieve the parameter and assign it to a variable (here `my_param`), as shown below for `PARAM_NAME`. The variable `my_param` can then be used in your module code.
    
    ```C
    int32_t my_param = 0;
    param_get(param_find("PARAM_NAME"), &my_param);
    ```
    
    > **Note** 如果在参数元数据中声明了 `PARAM_NAME`，则将设置其默认值，上述查找参数的调用应始终成功。
    
    `param_find()` is an "expensive" operation, which returns a handle that can be used by `param_get()`. If you're going to read the parameter multiple times, you may cache the handle and use it in `param_get()` when needed
    
    ```cpp
    # Get the handle to the parameter
    param_t my_param_handle = PARAM_INVALID;
    my_param_handle = param_find("PARAM_NAME");
    
    # Query the value of the parameter when needed
    int32_t my_param = 0;
    param_get(my_param_handle, &my_param);
    ```
    
    ## 参数元数据 {#parameter_metadata}
    
    PX4 uses an extensive parameter metadata system to drive the user-facing presentation of parameters, and to set the default value for each parameter in firmware.
    
    > **Tip** 正确的元数据对于在地面站获得良好的用户体验至关重要。
    
    Parameter metadata can be stored anywhere in the source tree as either **.c** or **.yaml** parameter definitions (the YAML definition is newer, and more flexible). Typically it is stored alongside its associated module.
    
    The build system extracts the metadata (using `make parameters_metadata`) to build the [parameter reference](../advanced/parameter_reference.md) and the parameter information used by ground stations.
    
    > **Warning** After adding a *new* parameter file you should call `make clean` before building to generate the new parameters (parameter files are added as part of the *cmake* configure step, which happens for clean builds and if a cmake file is modified).
    
    ### c 参数 Metadata {#c_metadata}
    
    The legacy approach for defining parameter metadata is in a file with extension **.c** (at time of writing this is the approach most commonly used in the source tree).
    
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
    
    The lines in the comment block are all optional, and are primarily used to control display and editing options within a ground station. The purpose of each line is given below (for more detail see [module_schema.yaml](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml)).
    
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
    
    ### YAML Metadata {#yaml_metadata}
    
    > **Note** At time of writing YAML parameter definitions cannot be used in *libraries*.
    
    YAML meta data is intended as a full replacement for the **.c** definitions. It supports all the same metadata, along with new features like multi-instance definitions.
    
    - The YAML parameter metadata schema is here: [validation/module_schema.yaml](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml).
    - An example of YAML definitions being used can be found in the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/module.yaml).
    #### Multi-Instance (Templated) Meta Data {#multi_instance_metadata}
    
    Templated parameter definitions are supported in [YAML parameter definitions](https://github.com/PX4/Firmware/blob/master/validation/module_schema.yaml) (templated parameter code is not supported).
    
    The YAML allows you to define instance numbers in parameter names, descriptions, etc. using `${i}`. For example, below will generate MY_PARAM_1_RATE, MY_PARAM_2_RATE etc.
    
        MY_PARAM_${i}_RATE:
                    description:
                        short: Maximum rate for instance ${i}
        
    
    The following YAML definitions provide the start and end indexes.
    
    - `num_instances` (default 1): Number of instances to generate (>=1)
    - `instance_start` (default 0): First instance number. If 0, `${i}` expands to [0, N-1]`.
    
    For a full example see the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/module.yaml)
    
    ## 更多信息
    
    - [Finding/Updating Parameters](https://docs.px4.io/master/en/advanced_config/parameters.html) (PX4 User Guide)
    - [Parameter Reference](../advanced/parameter_reference.md)