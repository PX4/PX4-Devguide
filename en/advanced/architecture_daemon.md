# Daemons

A daemon is a process running in the background. In NuttX a daemon process is a task, in POSIX (Linux / Mac OS) a daemon is a thread.

New daemons are created through the `px4_task_spawn()` command.

```C++
daemon_task = px4_task_spawn_cmd("commander",
			     SCHED_DEFAULT,
			     SCHED_PRIORITY_DEFAULT + 40,
			     3600,
			     commander_thread_main,
			     (char * const *)&argv[0]);
```

The arguments here are:

  * arg0: the process name, `commander`
  * arg1: the scheduling type (RR or FIFO)
  * arg2: the scheduling priority
  * arg3: the stack size of the new process or thread
  * arg4: the task / thread main function
  * arg5: a void pointer to pass to the new task, in this case holding the commandline arguments.

