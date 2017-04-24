# 常见问题



## 编译错误

### 内存溢出



> 使用FMUv4架构可以获得双倍的内存. 这一代的第一个可获得的板子是 [Pixracer](http://dev.px4.io/hardware-pixracer.html).




> 板子上可以下装的代码量受限于它自身的内存大小.当添加额外的模块或者代码时很有可能超过内存容量. 这将会导致“内存溢出”. 上游的版本总是可以编译的,但是依赖于一个开发者添加了什么，它有可能在本地造成溢出.

<div class="host-code"></div>

```sh
region `flash' overflowed by 12456 bytes
```

为了改正这个问题，要么使用最近的硬件，要么移除对你来说不是必要的模块. 配置在 [这里](https://github.com/PX4/Firmware/tree/master/cmake/configs). 为了移除一个模块, 可以直接注释掉它:

<div class="host-code"></div>

```cmake
#drivers/trone
```

## USB错误

### 程序烧录一直不成功

在Ubuntu中, 卸载调制解调器管理器:

```sh
sudo apt-get remove modemmanager
```
