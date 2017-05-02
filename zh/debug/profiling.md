---
translated_page: https://github.com/PX4/Devguide/blob/master/en/debug/profiling.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# Poor Man's Sampling Profiler

官网英文原文地址：https://dev.px4.io/advanced-profiling.html

本节介绍如何通过分析来评估PX4系统的性能。

本节的想法源于[Mark Callaghan和Domas Mituzas](https://dom.as/2009/02/15/poor-mans-contention-profiling/).



## 方法



PMSP是一个shell脚本，它会定期中断固件的运行，并对当前堆栈踪迹进行采样。采样的堆栈踪迹会被追加到一个文本文件中。一旦采样结束（通常要花一个小时，或者更长时间），收集的堆栈踪迹会被 *folded*。*folding*的结果是另一个含有相同堆栈踪迹的文本文件，相同的堆栈踪迹（比如那些在程序中相同点获得的）会合并到一起，并记录发生的次数。然后这个折叠的堆栈信息会被送入可视化脚本，这个脚本我们使用[FlameGraph - an open source stack trace visualizer](http://www.brendangregg.com/flamegraphs.html).

## 执行


脚本的位置在 `Debug/poor-mans-profiler.sh`。一旦开始运行，脚本会以指定的时间间隔运行指定数量的采样。收集的采样信息会存储在位于系统的临时文件夹(例如 `/tmp`)下的文本文件中。一旦采样结束，脚本会自动调用这个堆栈文件夹，输出信息会存储在临时文件夹下的相邻文件中。如果堆栈`folded`成功，脚本会调用 FlameGraph（火焰图） 脚本，并把结果保存在交互式的SVG文件中。请注意并不是所有的图片浏览器都支持交互式图片；推荐在网页浏览器中打开SVG结果。

FlameGraph脚本必须在 `PATH` 中，否则 PMSP 拒绝执行。

PMSP使用GDB收集堆栈踪迹。目前使用的是 `arm-none-eabi-gdb` ,未来可能会加入其他工具链。

为了能够将内存位置映射到符号上，脚本需要被引用到当前在目标上运行的可执行文件上。这通过选项 `--elf=<file>` 来实现。这个参数是一个指向当前执行的 ELF 的路径（相对于代码库的根路径）。  

用法示例:

```bash
./poor-mans-profiler.sh --elf=build_px4fmu-v4_default/src/firmware/nuttx/firmware_nuttx --nsamples=30000
```

注意每次运行脚本都会覆盖旧的堆栈。如果你想向旧的堆栈追加而不是重写，使用选项 `--append` ：

```bash
./poor-mans-profiler.sh --elf=build_px4fmu-v4_default/src/firmware/nuttx/firmware_nuttx --nsamples=30000 --append
```

正如人们怀疑的那样， `--append` 加上 `--nsamples=0` 会指示脚本在不访问目标的情况下重新生成SVG。


请阅读脚本深入理解脚本是如何工作的。

## 理解输出

下面是一个样例输出的截屏（并不是交互性的）

![FlameGraph Example](../../assets/flamegraph-example.png)


在火焰图上，水平方向表示堆栈帧，每个帧的宽度正比于它被采样的次数。反过来，函数最终被采样的次数正比于执行的持续时间频率。

## 可能的问题


该脚本是作为一种特殊解决方案开发的，所以仍然存在一些问题。使用它时请注意：


* 如果GDB发生故障，脚本可能无法检测到并继续运行。这种情况下显然不能产生可用的堆栈信息。为了避免这种情况，用户需要时不时地查看 `/tmp/pmpn-gdberr.log`文件，在里面包含着最近一次对GDB调用的stderr输出。在将来脚本会改为以安静模式调用，会通过exit code来指示问题。


* 有时GDB会在采样堆栈踪迹的时候卡死。这时候目标会一直暂停。解决办法是手动退出脚本，再用`--append` 选项重新运行脚本。将来，脚本会给每个GDB调用强制设置一个timeout。

* 不支持多线程环境。这不影响单核嵌入式目标，因为他们总是以单线程执行，但这种限制会使得分析器不能兼容许多其他的应用。在将来，会修改堆栈文件夹（stack folder译注：翻译存疑，stack folder可能指的是 相同堆栈踪迹的合并操作）以使每次采样支持多堆栈踪迹。
