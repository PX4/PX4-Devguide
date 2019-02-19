# 仿真调试

当模拟在主机上运行时，所有桌面开发工具都可用。

## CLANG Address Sanitizer (Mac OS, Linux)

CLANG Address Sanitizer 可以帮助查找对齐（总线）错误和其他内存错误，如段错误。 下面的命令设置了正确的编译选项。

```sh
make clean # 仅需在常规编译后，第一次运行 address sanitizer 时使用
PX4_ASAN=1 make px4_sitl jmavsim
```

## Valgrind

```sh
brew install valgrind
```

或

```sh
sudo apt-get install valgrind
```

> **Todo** 添加如何运行 Valgrind 的说明

## 开始组合

SITL 可以在附加调试器的情况下启动，也可以不附加调试器，并将 jMAVSim 或 Gazebo 作为模拟后端。 这将生成以下开始选项：

```sh
make px4_sitl_default jmavsim
make px4_sitl_default jmavsim___gdb
make px4_sitl_default jmavsim___lldb

make px4_sitl_default gazebo
make px4_sitl_default gazebo___gdb
make px4_sitl_default gazebo___lldb
```

其中最后一个参数是 <viewer\_model\_debugger> 三元组（使用三个下划线表示默认的 &#39;iris&#39; 模型）。 这将启动调试器并启动 SITL 应用程序。 为了终止调试器 shell 脚本并停止执行，请点击 ```CTRL-C```:

```gdb
Process 16529 stopped
* thread #1: tid = 0x114e6d, 0x00007fff90f4430a libsystem_kernel.dylib`__read_nocancel + 10, name = 'px4', queue = 'com.apple.main-thread', stop reason = signal SIGSTOP
    frame #0: 0x00007fff90f4430a libsystem_kernel.dylib`__read_nocancel + 10
libsystem_kernel.dylib`__read_nocancel:
->  0x7fff90f4430a <+10>: jae    0x7fff90f44314            ; <+20>
    0x7fff90f4430c <+12>: movq   %rax, %rdi
    0x7fff90f4430f <+15>: jmp    0x7fff90f3fc53            ; cerror_nocancel
    0x7fff90f44314 <+20>: retq
(lldb) 
```

为了没有驱动框架调度干扰调试会话 ```SIGCONT``` 应在 LLDB 和 GDB 中屏蔽：

```bash
(lldb) process handle SIGCONT -n false -p false -s false
```

或者在 GDB 下：

    (gdb) handle SIGCONT noprint nostop
    

之后，lldb 或 gdb 脚本的行为类似于正常会话，请参阅 ldb/gdbb 文档。

最后一个参数, <viewer\_model\_debugger> 三元组，实际上是传递到生成目录中，因此

```sh
make px4_sitl_default jmavsim___gdb
```

等价于

```sh
make px4_sitl_default   # 通过 cmake 配置
make -C build/px4_sitl_default jmavsim___gdb
```

使用以下方法获取生成目录中可用目标的完整列表：

```sh
make help
```

但为了您的方便，一个只有 <viewer\_model\_debugger> 三元组的列表可以用下面命令打印出来：

```sh
make list_vmd_make_targets
```

## 编译器优化

配置 `posix_sitl_*`时，对可执行文件和/或模块进行优化编译器选项（比如用 cmake 添加`add_executable` 或 `add_library` ），是种可以采取的手段。 当需要使用调试器或打印变量逐步执行代码时，这将非常方便，否则这些变量将被优化。

为此，请将环境变量 `PX4_NO_OPTIMIZATION` 设置为分号分隔的正则表达式列表，该列表与需要在不优化的情况下编译的目标相匹配。 当配置为&#39;t `posix_sitl_*` 时，将忽略此环境变量。

例如,

```sh
export PX4_NO_OPTIMIZATION='px4;^modules__uORB;^modules__systemlib$'
```

将抑制目标优化的有：platforms*\_posix**px4\_layer、modules**systemlib、modules**uORB、examples**px4\_simple\_app、modules**uORB*\_uORB\_tests 和 px4。

可以与这些正则表达式匹配的目标可以用命令打印想出来：

```sh
make -C build/posix_sitl_* list_cmake_targets
```