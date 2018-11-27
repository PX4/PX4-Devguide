# Poor Man's Sampling Profiler

이 섹션에서는 profiling을 통해서 PX4 시스템의 성능을 알아보는 방법에 대해서 알아봅니다.

이 아이디어는 [Mark Callaghan and Domas Mituzas](https://dom.as/2009/02/15/poor-mans-contention-profiling/)가 제안했습니다.

## 접근법

PMSP는 쉘 스크립트로 현재 stack trace을 샘플링하기 위해서 펌웨어 실행 중에 인터럽트를 거는 방식으로 동작합니다.
샘플링한 stack trace는 text 파일에 추가됩니다.
일단 샘플링이 끝나면(일반적으로 1시간 이상 걸림), 수집한 stack trace를 *folded* 합니다.
*folding* 한 결과로 동일한 stack trace를 포함하는 추가 text 파일이 생깁니다. 모든 유사한 stack trace(예로 프로그램내에서 동일한 지점에서 얻었던 것들)가 같이 합쳐지고 발생한 횟수가 기록된다는 것은 제외됩니다
folded stack은 시각화 스크립트의 입력으로 들어가는데 여기서는 [FlameGraph - 오픈소스 stack trace visualizer](http://www.brendangregg.com/flamegraphs.html)를 사용합니다.

## 구현

해당 스크립트는 `Debug/poor-mans-profiler.sh`에 위치합니다.
일단 실행되면 지정한 시간 구간으로 지정한 횟수만큼 샘플링을 수행합니다.
수집한 샘플은 system 임시 디렉토리(일반적으로 `/tmp`)에 있는 text 파일에 저장됩니다.
샘플링이 끝나면 스크립트는 자동으로 stack folder를 호출하고, 이 결과를 temp 디렉토리에 있는 파일에 저장합니다.
stack이 성공적으로 folded되면 스크립트는 FlameGraph 스크립트를 호출하고 결과를 SVG 파일에 저장합니다.
모든 이미지 뷰어 프로그램이 인터렉티브 이미지를 지원하지는 않습니다;
웹브라우져에서 결과로 나온 SVG 파일을 여는 것을 추천합니다.

FlameGraph 스크립트는 반드시 `PATH` 경로에 위치하고 있어야하며 그렇지 않은 경우 PMSP는 실행하지 않습니다.

PMSP는 GDB를 사용하여 stack trace를 수집합니다.
현재 `arm-none-eabi-gdb`를 사용하며, 다른 툴체인은 향후에 지원할 예정입니다.

메모리 위치를 심벌로 매핑하기 위해서 스크립트에서 실행가능한 파일을 지정하여 같이 타겟에서 실행해야 합니다.
`--elf=<file>` 옵션을 사용해서 실행하면, 저장소 루트의 상태 경로로 현재 실행 ELF의 위치를 지정합니다.

사용 예제:

```bash
./poor-mans-profiler.sh --elf=build/px4_fmu-v4_default/px4_fmu-v4_default.elf --nsamples=30000
```

스크립트를 매번 실행은 기존 stack을 덮어쓰게 된다는 것을 명심하세요. 기존 stack을 추가하기를 원한다면 `--append` 옵션을 사용하세요 :

```bash
./poor-mans-profiler.sh --elf=build/px4_fmu-v4_default/px4_fmu-v4_default.elf --nsamples=30000 --append
```

`--append`을 `--nsamples=0`와 함께 사용하면 스크립트는 타겟에 접근하지 않고 SVG를 다시 생성하라는 명령이 됩니다.

어떻게 동작하는지 깊게 이해하고자 한다면 스크립트를 읽어보세요.

## 출력 이해하기

예제 출력의 화면은 다음과 같습니다.(여기서는 상호작용이 없습니다) :

![FlameGraph 예제](../../assets/flamegraph-example.png)

flame 그래프에서 수평 레벨은 stack frame을 나타내며 각 frame의 폭은 샘플링하는 횟수에 비례합니다. 결국 특정 함수가 샘플링을 종료할때 실행 시간 빈도에 비례합니다.

## 가능한 이슈들

스크립트는 임시 방편으로 개발되어서 이슈가 발생할 수 있습니다.
사용하는 동안 잘 관찰해야 합니다:

* GDB가 오동작하는 경우, 스크립트가 이를 검출해 내지 못하고 계속 수행될 수 있습니다.
이 경우에는 사용가능한 stack이 만들어지지 않습니다.
이를 피할려면 사용자는 주기적으로 `/tmp/pmpn-gdberr.log` 파일을 확인해야만 합니다. 이 파일에는 가장 최근에 GDB의 호출에 대한 stderr 출력을 포함하고 있습니다.
향후에 스크립트는 quiet 모드에서 GDB를 호출하도록 수정해야만 합니다. 이렇게 하면 종료 코드를 통해 이슈를 찾아낼 수 있습니다.

* 가끔 GDB는 stack trace를 샘플링하는 동안 먹통이 되기도 합니다.
이런 경우 타겟은 멈추게 됩니다.
여기에 대한 해결책은 수동으로 스크립트를 정지시키고 `--append` 옵션으로 다시 실행시킵니다.
향후에 스크립트는 모든 GDB 호출에 대해서 타임아웃이 되도록 수정해야 합니다.

* 멀티쓰레드 환경 미지원
쓰레드가 한개인 환경에서 항상 실행되므로 단일 코어 임베디드 타겟인 경우에는 상관이 없습니다. 하지만 이런 제약은 프로파일러로 하여금 다른 어플리케이션과 호환성의 문제를 일으키게 됩니다.
향후에는 stack folder가 샘플마다 여러개 stack trace를 지원하도록 수정해야 합니다.
