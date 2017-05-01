---
translated_page: https://github.com/PX4/Devguide/blob/master/en/concept/middleware.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# PX4中间件

PX4中间件主要由内置传感器的驱动和基于发布-订阅（publish-subscribe）的中间件组成，其中发布-订阅中间件用于将这些传感器与[飞行控制](../concept/flight_stack.md)运行的应用程序进行通讯连接。

使用发布-订阅计划意味着:

* 系统是响应式的，即当有新的有效数据时系统能够立即更新
* 系统是完全并行运行的
* 系统组件能够在线程安全的方式下从任何地方使用数据

