# 外部模块


本教程描述了向PX4构建中添加外部模块的可能性。

外部模块可以使用与内部模块相同的模块，并且可以通过$\mu$ORB与内部模块交互

## 使用

- `EXTERNAL_MODULES_LOCATION` 需要指向一个与原生固件Firmware具有相同结构的目录(因此需包含一个称为`src`的目录)。
- 有两种方法：将现有模块(如examples/px4_simple_app)复制到外部目录，或者直接创建一个新的模块 。
- 重命名模块(包括CMakeLists.txt中的`MODULE`)或将其从现有的Firmware/cmake//config中移除。这是为了避免与内部模块发生冲突。
- 添加一个文件`$EXTERNAL_MODULES_LOCATION/CMakeLists.txt`，其内容包括：
```
set(config_module_list_external
    modules/<new_module>
    PARENT_SCOPE
    )
```
- 添加一行 `EXTERNAL` 到`modules/<new_module>/CMakeLists.txt`下的`px4_add_module`函数中, 例如像这样：

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

- 执行 `make posix EXTERNAL_MODULES_LOCATION=<path>`。可以使用任何其他的构建目标，但是构建目录必须是不存在的。如果它已经存在，你也可以在build文件夹中设置cmake变量。对于以后要增加的构建，就不需要再指定`EXTERNAL_MODULES_LOCATION`了。
