# 비주얼 스튜디오 코드 IDE (VSCode)

[비주얼 스튜디오 코드](https://code.visualstudio.com/)는 강력한 교차 플랫폼 소스 코드 편집기/통합 개발 환경이며, 우분투 18.04 LTS와 macOS(윈도우 는 곧 지원 예정)의 PX4 개발에 활용할 수 있습니다.

PX4 개발에 VSCode를 활용해야 하는 이유는 여러가지가 있습니다:

- 설치 시간이 *정말로* 단 몇분 밖에 걸리지 않습니다.
- 풍부한 확장기능 생태계 덕분에 PX4 개발시 C/C++ (단일 *cmake* 통합), *Python*, *Jinja2*, ROS 메시지, UAVCAN DSDL에 필요한 필요한 여러 도구를 쓸 수 있습니다: 
- Github와의 통합 기능이 우수합니다.

This topic explains how to setup the IDE and start developing.

> **Note** There are other powerful IDEs, but they typically take more effort to integrate with PX4. With *VScode*, configuration is stored in the PX4/Firmware tree ([Firmware/.vscode](https://github.com/PX4/Firmware/tree/master/.vscode)) so the setup process is as simple as adding the project folder.

## 선행 조건

You must already have installed the command line [PX4 developer environment](../setup/dev_env.md) for your platform and downloaded the *Firmware* source code repo.

## 설치 및 설정

1. [Download and install VSCode](https://code.visualstudio.com/) (you will be offered the correct version for your OS).
2. Open VSCode and add the PX4 source code:
    
   - Select *Open folder ...* option on the welcome page (or using the menu: **File > Open Folder**): ![폴더 열기](../../assets/vscode/welcome_open_folder.jpg)
   - A file selection dialog will appear. Select the PX4 **Firmware** directory and then press **OK**.
    
    The project files and configuration will then load into *VSCode*.

3. Press **Install All** on the *This workspace has extension recommendations* prompt (this will appear on the bottom right of the IDE). ![확장 기능 설치](../../assets/vscode/prompt_install_extensions.jpg)
    
    VSCode will open the *Extensions* panel on the left hand side so you can watch the progress of installation.
    
    ![VSCode 탐색기에 불러온 PX4
](../../assets/vscode/installing_extensions.jpg)

4. A number of notifications/prompts may appear in the bottom right corner
    
    > **Tip** If the prompts disappear, click the little "alarm" icon on the right of the bottom blue bar.

- If prompted to install a new version of *cmake*: 
   - Say **No** (the right version is installed with the [PX4 developer environment](../setup/dev_env.md)).
- If prompted to sign into *github.com* and add your credentials: 
   - This is up to you! It provides a deep integration between Github and the IDE, which may simplify your workflow.
- Other prompts are optional, and may be installed if they seem useful. <!-- perhaps add screenshot of these prompts -->

## PX4 빌드 {#building}

To build:

1. Select your build target ("cmake build config"): 
   - The current *cmake build target* is shown on the blue *config* bar at the bottom (if this is already your desired target, skip to next step). ![Cmake 빌드 대상 선택](../../assets/vscode/cmake_build_config.jpg)
   - Click the target on the config bar to display other options, and select the one you want (this will replace any selected target).
   - *Cmake* will then configure your project (see notification in bottom right). ![Cmake 설정 프로젝트](../../assets/vscode/cmake_configuring_project.jpg)
   - Wait until configuration completes. When this is done the notification will disappear and you'll be shown the build location: ![Cmake config project](../../assets/vscode/cmake_configuring_project_done.jpg).
2. You can then kick off a build from the config bar (select either **Build** or **Debug**). ![디버깅 또는 빌드 실행](../../assets/vscode/run_debug_build.jpg)

After building at least once you can now use [code completion](#code completion) and other *VSCode* features.

## PX4 디버깅 {#debugging_sitl}

SITL의 PX4를 디버깅하려면:

1. 측면 표시줄의 디버깅 아이콘을 선택(붉은색 표시)하여 디버깅 창을 표시하십시오.![디버깅 실행](../../assets/vscode/vscode_debug.jpg)

2. 상단 표시줄의 디버깅 드롭다운(자주색 상자)으로 디버깅 대상을 선택하십시오(예: *Debug SITL (Gazebo Iris)*).
    
    > **Note** 화면에 뜬(자주색 상자) 디버깅 대상은 여러분이 보유한 빌드 대상과 일치해야합니다(하단 표시줄의 황색 상자). 예를 들면, SITL 대상을 디버깅한다면, 빌드 대상 선택시 SITL을 넣어야합니다.

3. 디버깅 "play" 화살표를 눌러 디버깅을 시작하십시오(상단 표시줄의 디버깅 대상 옆 - 분홍색 상자에 있음).

While debugging you can set breakpoints, step over code, and otherwise develop as normal.

## 코드 자동 완성 {#code completion}

코드 자동 완성(과 기타 인텔리센스 마법 기능)이 동작하려면 설정을 활성화화하고 [코드를 빌드](#building)해야합니다.

이 과정이 끝나면 더이상 어떤 것도 하지 않아도 됩니다. 툴체인에서는 여러분이 입력한 심볼을 자동으로 찾아줍니다.

![인텔리센스](../../assets/vscode/vscode_intellisense.jpg)