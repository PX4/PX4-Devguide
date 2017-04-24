# 参数 & 配置

PX4使用参数子系统（实际就是浮点和整型数据的列表）和文本文件（用来配置Mixer混合器和启动脚本）来储存相关配置。

关于[系统启动](../advanced/system_startup.md) 和[机体参数配置](../airframes/adding_a_new_frame.md)的实现在其他章节有详细讲述。这部分主要是详细讨论参数子系统。

## 命令行的使用

PX4[系统控制台](../debug/system_console.md) 提供了 ```param``` 命令,可以对参数进行设置、访问、保存，以及从文件中导入和保存到文件。 

### 访问和设置参数

命令行param show 可以列出所有系统参数:

```sh
param show
```

参数名+字符可以选择对应的参数进行操作:

```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

### 导出和加载参数

一般的保存命令可以保存参数到默认的文件中:

```sh
param save
```

如果保存后面加上路径，将会保存参数到新的位置

```sh
param save /fs/microsd/vtol_param_backup
```

加载参数有两种方法:
 ```param load``` 
加载文件并用文件中的数据代替现有参数设置，最终把以前某个状态储存的数据一一复制过来
```param import``` 
这个命令更为精妙，它只改变与默认设置不同的参数。这个命令有重要的作用，比如在进行最初校准但不进行其他配置时，导入之前校准的参数就可以只改变校准数据而不对其他配置操作。

覆盖现有参数:

```sh
param load /fs/microsd/vtol_param_backup
```

合并现有参数和储存的参数 (储存文件中与默认参数不同的参数覆盖默认参数):

```sh
param import /fs/microsd/vtol_param_backup
```

## C / C++ API

PX4还有独立的C和C++接口访问配置数据。

<aside class="todo">
Discuss param C / C++ API.
</aside>

<div class="host-code"></div>

```C
int32_t param = 0;
param_get(param_find("PARAM_NAME"), &param);
```

##参数数据元

PX4使用一个参数数据元系统把参数展示给用户。正确的合适的数据元对地面站的用户体验有重要意义。

一段传统的数据元如下所示：

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

各行的作用：

```C++
/**
 * <title>
 *
 * <longer description, can be multi-line>
 *
 * @unit <the unit, e.g. m for meters>
 * @min <the minimum sane value. Can be overriden by the user>
 * @max <the maximum sane value. Can be overriden by the user>
 * @decimal <the minimum sane value. Can be overriden by the user>
 * @increment <the "ticks" in which this value will increment in the UI> 
 * @group <a title for parameters which form a group>
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```

