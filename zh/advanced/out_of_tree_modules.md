!REDIRECT "https://docs.px4.io/master/zh/advanced/out_of_tree_modules.html"

# 外部模块（Out-of-Tree）

外部模块为开发人员提供了一种便捷的机制，可以管理/分组他们想要添加（或更新）PX4固件的专有模块。 外部模块可以使用与内部模块相同的includes，并可以通过uORB与内部模块交互。

本主题说明如何将外部（“out of tree”）模块添加到PX4编译中。

> **Tip** 我们鼓励您尽可能将更改贡献给 PX4！

## 用法

要创建外部模块：

- 创建*外部目录*目录以对外部模块进行分组： 
  - This can be located anywhere outside of the **PX4-Autopilot** tree.
  - It must have the same structure as **PX4-Autopilot** (i.e. it must contain a directory called **src**).
  - 稍后我们使用` EXTERNAL_MODULES_LOCATION `来引用此目录。
- 将现有模块（例如**examples/px4_simple_app**）复制到外部目录，或直接创建新模块。
- Rename the module (including `MODULE` in **CMakeLists.txt**) or remove it from the existing PX4-Autopilot *cmake* build config. This is to avoid conflicts with internal modules.
- 在外部目录中添加文件** CMakeLists.txt **，内容为： 
      set(config_module_list_external
          modules/<new_module>
          PARENT_SCOPE
          )

- 在 `modules/&lt;new_module&gt;/CMakeLists.txt` 中添加一行 `EXTERNAL` `px4_add_module()`，例如：
  
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
      

<a id="uorb_message_definitions"></a>

## Out-of-Tree uORB Message Definitions

uORB messages can also be defined out-of-tree. For this, the `$EXTERNAL_MODULES_LOCATION/msg` folder must exist.

- 将所有新消息定义放在 `$EXTERNAL_MODULES_LOCATION/msg` 目录中。 这些新的树外消息定义的格式与任何其他 [uORB 消息定义](../middleware/uorb.md#adding-a-new-topic) 的格式相同。
- 将以下内容添加文件`$EXTERNAL_MODULES_LOCATION/msg/CMakeLists.txt`：
  
      set(config_msg_list_external
          <message1>.msg
          <message2>.msg
          <message3>.msg
          PARENT_SCOPE
          )
      
  
  其中` &lt;message#&gt;.msg `是要处理并用于生成uORB消息的uORB消息定义文件的名称。

The out-of-tree uORB messages will be generated in the same locations as the normal uORB messages. The uORB topic headers are generated in `<build_dir>/uORB/topics/`, and the message source files are generated in `<build_dir>/msg/topics_sources/`.

The new uORB messages can be used like any other uORB message as described [here](../middleware/uorb.md#adding-a-new-topic).

> **Warning** 树外的uORB消息定义不能与任何普通的uORB消息具有相同的名称。

<a id="building"></a>

## Building External Modules and uORB Messages

Execute `make px4_sitl EXTERNAL_MODULES_LOCATION=<path>`.

Any other build target can be used, but the build directory must not yet exist. If it already exists, you can also just set the *cmake* variable in the build folder.

For subsequent incremental builds `EXTERNAL_MODULES_LOCATION` does not need to be specified.