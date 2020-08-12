# 가난한 자의 샘플링 프로파일러

이 절에서는 프로파일링 평균 수치로 PX4 시스템 성능을 살펴보는 방법을 설명합니다.

## 접근

PSMP는 현재 스택을 추적하면서 샘플 값을 채취하기 위해 펌웨어 실행을 주기적으로 중단하며 동작하는 셸 스크립트입니다. 채취한 스택 추적 결과는 텍스트 파일에 넣습니다. 표본 데이터 채취가 끝나면(보통 몇시간 이상 걸림), 수집한 스택 추적 결과를 *접어둡니다*. *접어둔* 결과는 다른 텍스트 파일에 동일한 스택 추적 결과가 들어있지만, 서로 붙어있는 모든 유사 스택 추적 결과를 제외하고, 발생 횟수를 기록합니다. 접어둔 스택을 시각화 스크립트에 던져두는데 이 목적으로 우리가 사용하는 도구가 [FlameGraph - 오픈소스 스택 추적 시각화도구](http://www.brendangregg.com/flamegraphs.html)입니다.

## 기본 사용법

프로파일러 기본 사용법은 빌드 시스템에 있습니다. 예를 들면 다음 명령은 px4_fmu-v4pro 대상을 빌드한 후 추적 샘플 10000개를 만듭니다(필요한 경우 *FrameGraph*를 불러와서 경로를 추가하십시오).

    make px4_fmu-v4pro_default profile
    

샘플 채취 수를 조정하여 빌드 과정을 통제하고 싶다면 [구현](#implementation) 절을 살펴보십시오.

## 출력 내용 이해

예제 출력 화면은 아래에 있습니다(참고로 여기 화면이 대화식은 아닙니다):

![FlameGraph 예제](../../assets/flamegraph-example.png)

FlameGraph에서, 수평 단계는 스택 프레임을, 프레임의 높이는 샘플 채취 횟수에 비례합니다. 여기서 채취한 함수 실행 종료 횟수는 실행 주기 시간 길이에 비례합니다.

## 나타날 수 있는 문제

ad-hoc 솔루션으로 개발했기에 일부 문제가 있습니다. 사용중에 다음 내용을 확인하십시오:

* GDB가 제대로 동작하지 않으면, 스크립트는 GDB 발견에 실패하고 실행을 계속합니다. 이 경우, 명백하게 가용 스택이 나타나지 않습니다. 이 문제를 피하려면, 사용자는 최근 GDB 실행시 나타난 표준 오류 기록 파일 `/tmp/pmpn-gdberr.log`를 주기적으로 확인해야합니다. 나중에는 스크립트를 종료 코드로 문제를 나타내는 부분인 출력 동작없이 GDB를 실행하도록 수정할 예정입니다.

* 때로는 GDB가 스택 추적 표본 데이터를 수정하는 동안 GDB가 계속 멈춰있을 수가 있습니다. 이런 문제가 나타나면, 대상의 동작이 알 수 없는 이유로 끝납니다. 해결책은 스크립트를 일단 직접 멈추고 `--append` 옵션을 붙여 다시 실행하는 방법입니다. In the future the script should be modified to enforce a timeout for every GDB invocation.

* Multithreaded environments are not supported. This does not affect single core embedded targets, since they always execute in one thread, but this limitation makes the profiler incompatible with many other applications. In the future the stack folder should be modified to support multiple stack traces per sample.

## 구현 {#implementation}

The script is located at `Debug/poor-mans-profiler.sh`. Once launched, it will perform the specified number of samples with the specified time interval. Collected samples will be stored in a text file in the system temp directory (typically `/tmp`). Once sampling is finished, the script will automatically invoke the stack folder, the output of which will be stored in an adjacent file in the temp directory. If the stacks were folded successfully, the script will invoke the *FlameGraph* script and store the result in an interactive SVG file. Please note that not all image viewers support interactive images; it is recommended to open the resulting SVG in a web browser.

The FlameGraph script must reside in the `PATH`, otherwise PMSP will refuse to launch.

PMSP uses GDB to collect the stack traces. Currently it uses `arm-none-eabi-gdb`, other toolchains may be added in the future.

In order to be able to map memory locations to symbols, the script needs to be referred to the executable file that is currently running on the target. This is done with the help of the option `--elf=<file>`, which expects a path (relative to the root of the repository) pointing to the location of the currently executing ELF.

Usage example:

```bash
./poor-mans-profiler.sh --elf=build/px4_fmu-v4_default/px4_fmu-v4_default.elf --nsamples=30000
```

Note that every launch of the script will overwrite the old stacks. Should you want to append to the old stacks rather than overwrite them, use the option `--append`:

```bash
./poor-mans-profiler.sh --elf=build/px4_fmu-v4_default/px4_fmu-v4_default.elf --nsamples=30000 --append
```

As one might suspect, `--append` with `--nsamples=0` will instruct the script to only regenerate the SVG without accessing the target at all.

Please read the script for a more in depth understanding of how it works.

## 기여자

[Mark Callaghan과 Domas Mituzas](https://dom.as/2009/02/15/poor-mans-contention-profiling/)의 아이디어입니다.