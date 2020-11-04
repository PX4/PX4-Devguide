# 모듈과 명령어 참고

다음 페이지에서는 PX4 모듈, 드라이버, 명령어를 설명합니다. 제공 기능, 고수준 구현 개요, 명령행 인터페이스 사용법을 설명합니다.

> **Note** **이 문서는 소스 코드에서 자동으로 추출하여 만들었으며** 최신 모듈 문서입니다.

완전한 목록은 아니며, NuttX에서는 (`free` 같은) 추가 명령어를 제공합니다. 모든 가용 명령어를 살펴보시려면 `help`를 사용하십시오. 대부분의 경우 `command help` 명령에서 사용법을 출력합니다.

Since this is generated from source, errors must be reported/fixed in the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository. 문서 페이지는 Firmware 디렉터리 루트에서 다음 명령을 실행하야 만들 수 있습니다:

    make module_documentation
    

이 명령을 통해 `modules` 디렉터리에 내용을 추출한 파일을 기록합니다.

## 카테고리

- [명령어](modules_command.md)
- [통신](modules_communication.md)
- [조종 장치](modules_controller.md)
- [드라이버](modules_driver.md)
- [추정자](modules_estimator.md)
- [모의 시험](modules_simulation.md)
- [시스템](modules_system.md)
- [서식](modules_template.md)