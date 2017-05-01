---
translated_page: https://github.com/PX4/Devguide/commits/master/en/advanced/architecture_daemon.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 守护进程


守护进程`daemon`是运行在后台的进程。
在NuttX中守护进程是一个任务，在POSIX(Linux/Mac OS)中是一个线程

```C++
daemon_task = px4_task_spawn_cmd("commander",
			     SCHED_DEFAULT,
			     SCHED_PRIORITY_DEFAULT + 40,
			     3600,
			     commander_thread_main,
			     (char * const *)&argv[0]);
```

以下是参数：
- arg0: 进程名 `commander`
- arg1: 调度类型（RR or FIFO）the scheduling type (RR or FIFO)
- arg2: 调度优先级
- arg3: 新进程或线程堆栈大小
- arg4: 任务/线程主函数
- arg5: 一个void指针传递给新任务,在这种情况下是命令行参数
