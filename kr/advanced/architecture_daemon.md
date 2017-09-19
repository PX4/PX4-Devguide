# Daemons

daemon은 백그라운드로 실행되는 일종의 프로세스입니다. NuttX에서 daemon 프로세스는 일종의 task고, POSIX(Linux / Mac OS)에서 daemon은 일종의 thread입니다.

새로운 daemon은 `px4_task_spawn_cmd()` 명령을 통해 생성할 수 있습니다.

```C++
daemon_task = px4_task_spawn_cmd("commander",
			     SCHED_DEFAULT,
			     SCHED_PRIORITY_DEFAULT + 40,
			     3600,
			     commander_thread_main,
			     (char * const *)&argv[0]);
```

각 인자들은 :

  * arg0: process 이름, `commander`
  * arg1: 스케쥴링 타입 (RR 혹은 FIFO)
  * arg2: 스케쥴링 우선순위
  * arg3: 새로운 process나 thread의 스택 사이즈
  * arg4: 해당 task / thread 메인 함수
  * arg5: 새로운 task로 전달하는 void 포인터, 이 경우 commandline 인자를 가짐
