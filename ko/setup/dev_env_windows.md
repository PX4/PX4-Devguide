# 윈도우 환경 설치 방법

윈도우에서 PX4 를 개발하려면, [윈도우용 Cygwin 툴체인](../setup/dev_env_windows_cygwin.md)에서 다음 절차를 따르십시오.

> **Tip** *Cygwin 툴체인에서는* NuttX/픽스호크, jMAVSim 모의시험 환경대상 빌드를 지원합니다. [다른 대상](/setup/dev_env.md#supported-targets)을 빌드하고자 한다면, [우분투 리눅스](http://ubuntu.com) 이중 부팅 시스템 설치를 고려하십시오.

## 추가 도구

빌드/모의시험 툴체인을 설치하고 난 후, 기타 유용한 "일반 개발" 도구 정보를 살펴보려면 [추가 도구](../setup/generic_dev_tools.md)를 실펴보십시오.

## 다음 단계

환경 구성이 끝나면, [빌드 설명서](../setup/building_px4.md)로 계속 진행하십시오.

## 기타 윈도우 툴체인

일부 개발자에게 흥미로울 법한 수많은 기존/대안이 있습니다. A comparison of the options is provided below.

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

- 2018년도 도입
- 간단한 설치
- 휴대성 있음

|

- 완전한 리눅스 기능을 갖춤
- CPU와 RAM 자원을 집중 소모
- 집약적 디스크 공간 소모

|

- 모의시험 환경 인터페이스 "해부" 필요
- 윈도우 10 전용
- 가상 머신

|