# 外部模块（Out-of-Tree）

外部模块为开发人员提供了一种便捷的机制，可以管理/分组他们想要添加（或更新）PX4固件的专有模块。 外部模块可以使用与内部模块相同的includes，并可以通过uORB与内部模块交互。

本主题说明如何将外部（“out of tree”）模块添加到PX4编译中。

> **Tip** 我们鼓励您尽可能将更改贡献给 PX4！

## 用法

要创建外部模块：

- 创建*外部目录*目录以对外部模块进行分组： 
  - 它可以位于 **Firmware** 目录树之外的任何位置。
  - 它必须与** Firmware **具有相同的目录结构（即它必须包含名为** src **的目录）。
  - 稍后我们使用` EXTERNAL_MODULES_LOCATION `来引用此目录。
- 将现有模块（例如**examples/px4_simple_app**）复制到外部目录，或直接创建新模块。
- 重命名模块（包括 **CMakeLists.txt** 中的 `MODULE`）或从现有的Firmware *cmake* 构建配置中删除它。 这是为了避免与内部模块冲突。
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
      

## Out-of-Tree uORB 消息定义 {#uorb_message_definitions}

uORB消息也可以在树外定义。 为此，必须存在 `$EXTERNAL_MODULES_LOCATION/msg` 文件夹。

- 将所有新消息定义放在 `$EXTERNAL_MODULES_LOCATION/msg` 目录中。 这些新的树外消息定义的格式与任何其他 [uORB 消息定义](../middleware/uorb.md#adding-a-new-topic) 的格式相同。
- 将以下内容添加文件`$EXTERNAL_MODULES_LOCATION/msg/CMakeLists.txt`：
  
      set(config_msg_list_external
          <message1>.msg
          <message2>.msg
          <message3>.msg
          PARENT_SCOPE
          )
      
  
  其中` &lt;message#&gt;.msg `是要处理并用于生成uORB消息的uORB消息定义文件的名称。

树外uORB消息将在与正常uORB消息相同的位置生成。 Uorb主题标题在` &lt;build_dir&gt; / uORB / topics / `中生成，消息源文件在` &lt;build_dir&gt; / msg / topics_sources / `中生成。

新的uORB消息可以像任何其他uORB消息一样使用，如[这里](../middleware/uorb.md#adding-a-new-topic)所述。

> **Warning** 树外的uORB消息定义不能与任何普通的uORB消息具有相同的名称。

## 构建外部模块和 uORB 消息 {#building}

执行` make px4_sitl EXTERNAL_MODULES_LOCATION = &lt;path&gt; `。

可以使用任何其他构建目标，但构建目标目录必须不存在。 如果它已经存在，您还可以在构建文件夹中设置* cmake *变量。

对于后续增量构建，不需要指定` EXTERNAL_MODULES_LOCATION `。