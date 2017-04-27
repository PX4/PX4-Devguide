# 测试与持续集成

PX4提供大量的测试和持续集成。 本页提供概述。

## 在本地机器上测试
下面这条命令足够打开一个带有运行中的PX4端口的新shell。

```
make posix_sitl_shell none
```

shell可以用这个例子运行单元测试:

```
pxh> tests mixer
```

另一种选择也可以从bash中运行以下命令运行完整的单元测试:

```
make tests
```

## 在云端测试和持续集成


