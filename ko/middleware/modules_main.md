# 모듈과 명령어 참고

다음 페이지에서는 PX4 모듈, 드라이버, 명령어를 설명합니다. 제공 기능, 고수준 구현 개요, 명령행 인터페이스 사용법을 설명합니다.

> **Note** **이 문서는 소스 코드에서 자동으로 추출하여 만들었으며** 최신 모듈 문서입니다.

완전한 리스트는 아니며 게다가 NuttX는 추가적인 명령러를 제공합니다(`free`). `help` 명령어를 사용하는한 명령어를 모두 보여 줍니다. 대부분의 경우에 `command help`는 사용법을 출력해줄 것입니다.

이거은 소스코드에서 생성되기 때문에, 에러는 [Firmware](https://github.com/PX4/Firmware) 저장소에 반드시 리포팅되고 수정되어야 합니다. 문서 페이지는 펌웨어 디렉토리의 루트에서 다음의 명령어를 입력함으로써 생성할 수 있습니다.

    make module_documentation
    

`modules` 디렉토리에 파일들이 생성될 것입니다.

## 카테고리

- [Command](modules_command.md)
- [Communication](modules_communication.md)
- [Controller](modules_controller.md)
- [Driver](modules_driver.md)
- [Estimator](modules_estimator.md)
- [Simulation](modules_simulation.md)
- [System](modules_system.md)
- [Template](modules_template.md)