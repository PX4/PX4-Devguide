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
    
    此外，C++ 版本在 RAM 方面也具有更好的类型安全性和更少的开销。 缺点是参数名称必须在编译时知道，而 C语言 API 可以将动态创建的名称作为字符串。
    
    ### C++ API
    
    C++ api 提供宏将参数声明为 *class 属性*。 您可以添加一些 "样板" 代码，以定期侦听与 *any* 参数更新相关的 [uORB topic](../middleware/uorb.md) 中的更改。 然后，框架代码（无形地）处理跟踪影响参数属性并保持它们同步的 uORB 消息。 在代码的其余部分中，您只需使用定义的参数属性，它们将始终是最新的!
    
    First include **px4_platform_common/module_params.h** in the class header for your module or driver (to get the `DEFINE_PARAMETERS` macro):
    
    ```cpp
    #include <px4_platform_common/module_params.h>
    ```
    
    从 `ModuleParams` 派生类，并使用 `DEFINE_PARAMETERS` 指定参数及其关联参数属性的列表。 参数的名称必须与其参数元数据定义相同。
    
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
    
    使用样板更新 CPP 文件，以检查与参数更新相关的 uORB 消息。
    
    首先包括访问 uORB parameter_update 消息的标头:
    
    ```cpp
    #include <uORB/topics/parameter_update.h>
    ```
    
    在模块驱动程序启动时订阅更新消息，在停止时取消订阅。 `orb_subscribe()` 返回 `parameter_update_sub` 是我们可以用来引用此特定订阅的句柄。
    
    ```cpp
    # Subscribe to parameter_update message
    int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    ...
    # Unsubscribe to parameter_update messages
    orb_unsubscribe(parameter_update_sub);
    ```
    
    在代码周期性调用 `parameters_update(parameter_update_sub);` ，检查是否有更新（本模板）：
    
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
    
    在上面的方法中：
    
    - `orb_check()` 告诉我们是否有 *任何* 更新 `param_update` 的 uorb 消息 (但不是受影响的参数)，并设置 `updated` bool。
    - 如果更新了 "某些" 参数，我们会将更新复制到 `parameter_update_s` (`param_upd`)
    - 调用 `ModuleParams::updateParams()`。 在此检查 `DEFINE_PARAMETERS` 列表中列出的特定参数属性是否需要更新，然后在需要时进行更新。
    - 当 `force=True` 时，此示例不调用 `Module::p arameters_update()`。 如果您有其他需要设置公共模式的值，则是将它们包含在函数中，并在初始化过程中使用 `force=True` 调用它一次。
    
    然后，参数属性 (`_sys_autostart` 和 `_att_bias_max` 在本例中) 可用于表示参数，并将在参数值更改时进行更新。
    
    > **Tip** [Application/Module templateet](../apps/module_template.md) 使用新型 C++ API，但不包括 [parameter 元目录 ](#parameter_metadata)。
    
    ### C API
    
    C API 可以在模块和驱动程序中使用。
    
    首先包括参数 API:
    
    ```C
    #include <parameters/param.h>
    ```
    
    然后检索该参数并将其分配给变量 (此处 `my_param`)，如下所示，用于 `PARAM_NAME`。 然后，可以在模块代码中使用变量 `my_param`。
    
    ```C
    int32_t my_param = 0;
    param_get(param_find("PARAM_NAME"), &my_param);
    ```
    
    > **Note** 如果在参数元数据中声明了 `PARAM_NAME`，则将设置其默认值，上述查找参数的调用应始终成功。
    
    `param_find()` 是一个 "昂贵" 的操作，它返回可供 `param_get()` 使用的句柄。 如果要多次读取该参数，可以缓存句柄，并在需要时在 `param_get()` 中使用
    
    ```cpp
    # Get the handle to the parameter
    param_t my_param_handle = PARAM_INVALID;
    my_param_handle = param_find("PARAM_NAME");
    
    # Query the value of the parameter when needed
    int32_t my_param = 0;
    param_get(my_param_handle, &my_param);
    ```
    
    ## 参数元数据 {#parameter_metadata}
    
    PX4 使用广泛的参数元数据系统来驱动面向用户的参数表示，并为固件中的每个参数设置默认值。
    
    > **Tip** 正确的元数据对于在地面站获得良好的用户体验至关重要。
    
    Parameter metadata can be stored anywhere in the source tree as either **.c** or **.yaml** parameter definitions (the YAML definition is newer, and more flexible). 通常，它与关联的模块一起存储。
    
    构建系统提取 metadata（使用 `make parameters_metadata`）来构建 [parameter reference ](../advanced/parameter_reference.md) 和地面站使用的参数信息。
    
    ### c Parameter Metadata {#c_metadata}
    
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