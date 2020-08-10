# PX4용 MCU Eclipse/J-Link 디버깅

이 주제에서는 [MCU Eclipse](https://gnu-mcu-eclipse.github.io/)와 NuttX에서 동작하는 PX4 디버깅장비 *Segger Jlink adapter*를 설치하고 활용하는 방법을 설명합니다.


## 필요한 하드웨어

- [J-Link EDU Mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/)
- Segger JLink를 비행 컨트롤러로 연결하는 어댑터 [SWD (JTAG) 하드웨어 디버깅 인터페이스](../debug/swd_debug.md)(디버깅 포트).
- 마이크로 USB 케이블
- 대상 하드웨어에 연결할 적당한 케이블.

## 설치

### PX4

일반 지침을 따라 PX4를 설치하십시오:
- 해당 플랫폼에 [PX4 개발자 환경/툴체인을 설치](../setup/dev_env.md) 하십시오(예: 리눅스의 경우 [우분투 LTS / 데비안 리눅스 개발 환경](../setup/dev_env_linux_ubuntu.md) 참고).
- [PX4를 다운로드](../setup/building_px4.md) 하고, 명령행에서 별도로 빌드하십시오.

### Eclipse

*Eclipse*를 설치하려면:
1. [Eclipse CDT for C/C++ Developers](https://github.com/gnu-mcu-eclipse/org.eclipse.epp.packages/releases/) (MCU Github)를 다운로드하십시오.
1. Eclipse 폴더 압축을 해제하고 어딘가에든 복사하십시오(별도의 설치 스크립트 실행은 필요하지 않습니다).
1. *Eclipse*를 실행하고 초기 작업 영역 위치를 선택하십시오.

### Segger Jlink 도구

To install the *Segger Jlink* tools:
1. Download and run the [J-Link Software and Documentation Pack](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack) for your OS (Windows and Linux packages available).
   - On Linux the tools are installed in **usr/bin**.

For more information, see: https://gnu-mcu-eclipse.github.io/debug/jlink/install/

## First Use

1. Connect the *Segger JLink* to the host computer and the [flight controller debug port](../debug/swd_debug.md) (via an adapter).
1. Power the flight controller.
1. Run *Eclipse*.
1. Add a source by choosing **File > Import > C/C++ > Existing Code as Makefile Project** and click **Next**.
1. Point it to the **Firmware** folder and give it a name, then select *ARM Cross GCC* in the *Toolchain for Indexer Settings* and click **Finish**. Import takes a while, wait for it to complete.
1. Set the MCU settings: right-click on the top-level project in the Project Explorer, select *Properties* then under MCU choose *SEGGER J-Link Path*. Set it as shown in the screenshot below. ![Eclipse: Segger J-Link 경로](../../assets/debug/eclipse_segger_jlink_path.png)
1. Update packs:
   - Click the small icon on the top right called *Open Perspective* and open the *Packs* perspective. ![Eclipse: Workspace](../../assets/debug/eclipse_workspace_perspective.png)
   - Click the **update all** button.

     > **Tip** This takes a VERY LONG TIME (10 minutes). Ignore all the errors about missing packages that pop up.

     ![Eclipse: Workspace Packs Perspective](../../assets/debug/eclipse_packs_perspective.jpg)
   - The STM32Fxx devices are found in the Keil folder, install by right-clicking and then selecting **install** on the according device for F4 and F7.
1. Setup debug configuration for target:
   - Right click project and open the *Settings* (menu: **C/C++ Build > Settings**)
   - Choose the *Devices* Tab, *Devices* section (Not *Boards*).
   - Find the FMU chip you wish to debug.

   ![Eclipse: Select FMU in settings](../../assets/debug/eclipse_settings_devices_fmu.png)
1. Select debug configurations with the small drop-down next to the bug symbol: ![Eclipse: Debug config](../../assets/debug/eclipse_settings_debug_config.png)
1. Then select *GDB SEGGER J-Link Debugging* and then the **New config** button on the top left. ![Eclipse: GDB Segger Debug config](../../assets/debug/eclipse_settings_debug_config_gdb_segger.png)
1. Setup build config:
   - Give it a name and set  the *C/C++ Application* to the corresponding **.elf** file.
   - Choose *Disable Auto build* > **Note** Remember that you must build the target from the command line before starting a debug session.

   ![Eclipse: GDB Segger Debug config](../../assets/debug/eclipse_settings_debug_config_gdb_segger_build_config.png)
1. The *Debugger* and *Startup* tabs shouldn’t need any modifications (just verify your settings with the screenshots below)

   ![Eclipse: GDB Segger Debug config: debugger tab](../../assets/debug/eclipse_settings_debug_config_gdb_segger_build_config_debugger_tab.png) ![Eclipse: GDB Segger Debug config: startup tab](../../assets/debug/eclipse_settings_debug_config_gdb_segger_build_config_startup_tab.png)


## 문제 해결

### 대상 CPU가 패키지 관리자에 없음

대상 CPU가 패키지 관리자에 없다면 레지스터 뷰가 동작하는지 확인하기 위해 다음 단계를 거쳐야 합니다.

> **Tip** 보통 일어나서는 안될 일입니다(만, STM F7 컨트롤러 연결에 대해 보고가 들어온 적이 있습니다).

*주변 장치 보기*에서 빠진 SVD 파일을 추가하려면:
1. Find out where MCU Eclipse stores its packages (**Preferences > C/C++ > MCU Packages**): ![Eclipse: MCU Packages](../../assets/debug/eclipse_mcu_packages.png)
2. Download missing packages from: http://www.keil.com/dd2/Pack/
3. Open downloaded pack with a decompression tool, and extract the **.SVD** files from: **/CMSIS/SVD**.
4. Select desired **.SVD** file in: **Debug Options > GDB SEGGER JLink Debugging > SVD Path** ![Eclipse: SVD 파일 경로](../../assets/debug/eclipse_svd_file_path.png)
