# 윈도우 환경 설치 방법

윈도우에서 PX4 를 개발하려면, [윈도우용 Cygwin 툴체인](../setup/dev_env_windows_cygwin.md)에서 다음 절차를 따르십시오.

> **Tip** *Cygwin 툴체인에서는* NuttX/픽스호크, jMAVSim 모의시험 환경대상 빌드를 지원합니다. [다른 대상](/setup/dev_env.md#supported-targets)을 빌드하고자 한다면, [우분투 리눅스](http://ubuntu.com) 이중 부팅 시스템 설치를 고려하십시오.

## 추가 도구

After setting up the build/simulation toolchain, see [Additional Tools](../setup/generic_dev_tools.md) for information about other useful "general development" tools.

## 다음 단계

환경 구성이 끝나면, [빌드 설명서](../setup/building_px4.md)로 계속 진행하십시오.

## 기타 윈도우 툴체인

There are a number of other legacy/alternative solutions that may be of interest to some developers. A comparison of the options is provided below.

> **Note** The [Cygwin Toolchain](../setup/dev_env_windows_cygwin.md) is the only one that is supported by the PX4 dev team. It is regularly tested as part of our continuous integration system and is known to be better performing than the other alternatives.

|                         | [Cygwin Toolchain](../setup/dev_env_windows_cygwin.md) **(Supported)** | [Virtual Machine Toolchain](../setup/dev_env_windows_vm.md) | [Bash on Windows Toolchain](../setup/dev_env_windows_bash_on_win.md) |
| ----------------------- | ---------------------------------------------------------------------- | ----------------------------------------------------------- | -------------------------------------------------------------------- |
| Installation            | MSI installer or Script                                                | Script                                                      | Script                                                               |
| Native binary execution | yes                                                                    | no                                                          | no                                                                   |
| Performance             | ++                                                                     | --                                                          | -                                                                    |
| ARM Targets             | ++ (quick)                                                             | + (VM USB)                                                  | +                                                                    |
| Simulation jMAVSim      | ++                                                                     | +                                                           | +                                                                    |
| Simulation gazebo       | - (not yet)                                                            | + (slow)                                                    | + (slow)                                                             |
| Support                 | +                                                                      | ++ (Linux)                                                  | +/-                                                                  |
| Comments                |                                                                        |                                                             |                                                                      |

- New in 2018
- Slim setup
- Portable

|

- Full Linux features
- CPU & RAM intensive
- Disk space intensive

|

- Simulation UI is a "hack".
- Windows 10 only
- Essentially a VM

|