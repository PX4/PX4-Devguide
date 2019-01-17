# 参数设置

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

### 导出和加载参数

您可以保存自上次将所有参数重置为其固件定义的默认值以来 *touched* 的任何参数（这包括已更改的任何参数，即使这些参数已更改为默认值）。

标准的 `param save ` 命令将参数存储在当前默认文件中:

```sh
param save
```

如果提供了参数，它将将参数存储到这个新位置:

```sh
param save /fs/microsd/vtol_param_backup
```

有两个不同的命令可用于 *load* 参数:

- `param load ` 首先将所有参数完全重置为默认值，然后用存储在文件中的任何值覆盖参数值。
- `param import ` 只是用文件中的值覆盖参数值，然后保存结果（即有效调用 `param save</0 >）。</li>
</ul>

<p><code>load` 有效地将参数重置为保存参数时的状态（我们说 "有效"，因为保存在文件中的任何参数都将被更新，但其他参数可能具有与参数文件）。</p> 
    相比之下，`import` 将文件中的参数与车辆的当前状态合并。 例如，这可以用来只导入包含校准数据的参数文件，而不覆盖系统配置的其余部分。
    
    这两种情况的示例如下所示:
    
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
    
    参数名称不得超过 16个 ASCII 字符。
    
    按照惯例，组中的每个参数都应共享相同的 (有意义的) 字符串前缀，后跟下划线，`MC_` 和 `FW_` 用于与多旋翼或固定翼系统具体相关的参数。 此惯例不强制执行。
    
    该名称必须在代码和 [parameter metadatata](#parameter_metadata) 中匹配，才能正确地将参数与其元数据（包括固件中的默认值）相关联。
    
    ## C / C++ API
    
    有单独的 C 和 C++ 的 API 可用于从 PX4 模块和驱动程序中访问参数值。
    
    API 之间的一个重要区别是，C++ 版本具有更有效的标准化机制，可与参数值的更改（即来自 GCS 的更改）同步。
    
    同步很重要，因为参数可以随时更改为另一个值。 您的代码应该 *always* 使用参数存储中的当前值。 如果无法获取最新版本，则需要在更改参数后重新启动（使用 `@reboot_required` 元数据设置此要求）。
    
    此外，C++ 版本在 RAM 方面也具有更好的类型安全性和更少的开销。 The drawback is that the parameter name must be known at compile-time, while the C API can take a dynamically created name as a string.
    
    ### C++ API
    
    The C++ API provides macros to declare parameters as *class attributes*. You add some "boilerplate" code to regularly listen for changes in the [uORB Topic](../middleware/uorb.md) associated with *any* parameter update. Framework code then (invisibly) handles tracking uORB messages that affect your parameter attributes and keeping them in sync. In the rest of the code you can just use the defined parameter attributes and they will always be up to date!
    
    First include **px4_module_params.h** in the class header for your module or driver (to get the `DEFINE_PARAMETERS` macro):
    
    ```cpp
    #include <px4_module_params.h>
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
    
    - `orb_check()` tells us if there is *any* update to the `param_update` uORB message (but not what parameter is affected) and sets the `updated` bool.
    - If there has been "some" parameter updated, we copy the update into a `parameter_update_s` (`param_upd`)
    - Then we call `ModuleParams::updateParams()`. This "under the hood" checks if the specific parameter attributes listed in our `DEFINE_PARAMETERS` list need updating, and then does so if needed.
    - This example doesn't call `Module::parameters_update()` with `force=True`. If you had other values that needed to be set up a common pattern is to include them in the function, and call it once with `force=True` during initialisation.
    
    The parameter attributes (`_sys_autostart` and `_att_bias_max` in this case) can then be used to represent the parameters, and will be updated whenever the parameter value changes.
    
    > **Tip** The [Application/Module Template](../apps/module_template.md) uses the new-style C++ API but does not include [parameter metadata](#parameter_metadata).
    
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
    
    > **Note** If `PARAM_NAME` was declared in parameter metadata then its default value will be set, and the above call to find the parameter should always succeed.
    
    `param_find()` is an "expensive" operation, which returns a handle that can be used by `param_get()`. If you're going to read the parameter multiple times, you may cache the handle and use it in `param_get()` when needed
    
    ```cpp
    # Get the handle to the parameter
    param_t my_param_handle = PARAM_INVALID;
    my_param_handle = param_find("PARAM_NAME");
    
    # Query the value of the parameter when needed
    int32_t my_param = 0;
    param_get(my_param_handle, &my_param);
    ```
    
    ## Parameter Meta Data {#parameter_metadata}
    
    PX4 uses an extensive parameter metadata system to drive the user-facing presentation of parameters, and to set the default value for each parameter in firmware.
    
    > **Tip** Correct meta data is critical for good user experience in a ground station.
    
    Parameter metadata can be stored anywhere in the source tree, in a file with extension **.c**. Typically it is stored alongside its associated module.
    
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