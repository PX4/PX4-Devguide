---
translated_page: https://github.com/PX4/Devguide/blob/master/en/concept/architecture.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 结构预览

PX4由两个层次组成：一是[飞行控制栈(flight stack)](../concept/flight_stack.md)，即自驾仪的软件解决方案，二是[中间件](../concept/middleware.md)，一种可以支持任意类型自主机器人的通用机器人中间件。


所有的[无人机机型](../airframes/architecture.md)，事实上所有的包括船舶在内的机器人系统，都具有同一代码库。整个系统设计是[反应式(reactive)](http://www.reactivemanifesto.org)的，这意味着：

- 所有的功能被划分为可替换部件
- 通过异步消息传递进行通信
- 该系统可以应对不同的工作负载

除了这些运行时考虑之外，其模块化最大限度地提高了系统的[可重用性](https://en.wikipedia.org/wiki/Reusability)。

## 顶层软件结构

下面每一块都是单独的模块，不论是在代码，依赖性甚至是在运行时都是独立的。每个箭头是一种通过 [uORB](../middleware/uorb.md)进行发布/订阅调用的连接。

> ** 须知：** PX4结构允许其即使是在运行时，也可以快速方便地交换各个单独的模块。


控制器/混控器是针对于特定机型的（如多旋翼，垂直起降或其他飞机），但是顶层任务管理模块如`commander` 和`navigator` 是可以在不同平台共享的。

![Architecture](../../assets/diagrams/PX4_Architecture.png)

> ** Info ** This flow chart can be updated from [here](https://drive.google.com/file/d/0Byq0TIV9P8jfbVVZOVZ0YzhqYWs/view?usp=sharing) and open it with draw.io Diagrams.

## 地面站通讯结构


与地面站（GCS）之间的交互是通过一种“商业逻辑”应用程序来处理的，包括如 commander( 一般命令与控制，例如解锁 ) ， navigator ( 接受任务并将其转为底层导航的原始数据 ) 和 mavlink 应用， mavlink 用于接受 MAVLink 数据包并将其转换为板载 uORB 数据结构。这种隔离方式使架构更为清晰，可以避免系统对 MAVLink  过于依赖 。 MAVLink 应用也会获取大量的传感器数据和状态估计值，并将其发送到地面站。


[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIG1hdmxpbmstLS1jb21tYW5kZXI7XG4gIG1hdmxpbmstLS1uYXZpZ2F0b3I7XG4gIHBvc2l0aW9uX2VzdGltYXRvci0tPm1hdmxpbms7XG4gIGF0dGl0dWRlX2VzdGltYXRvci0tPm1hdmxpbms7XG4gIG1peGVyLS0-bWF2bGluazsiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIG1hdmxpbmstLS1jb21tYW5kZXI7XG4gIG1hdmxpbmstLS1uYXZpZ2F0b3I7XG4gIHBvc2l0aW9uX2VzdGltYXRvci0tPm1hdmxpbms7XG4gIGF0dGl0dWRlX2VzdGltYXRvci0tPm1hdmxpbms7XG4gIG1peGVyLS0-bWF2bGluazsiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)
