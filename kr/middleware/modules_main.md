
# Modules & Commands 레퍼런스
이번 페이지에서는 PX4 모듈, 드라이버와 명령에 대해서 다룹니다. 제공하는 기능, 하이레벨 구현 개요 및 커맨드라인 인터페이스 사용법에 대해서 설명합니다.

> **Note** **이것은 소스코드에서 자동으로 생성되며**
> 가장 최근 모듈 문서를 포함합니다.

이것은 전체 목록은 아니고 NuttX가 제공하는 추가 명령들을 제공합니다.(`free`같은) `help`를 콘솔에서 사용하면 사용가능한 모듈 명령의 목록을 얻을 수 있고 대부분 경우 `command help`는 사용법을 출력합니다.

소스코드에서 생성되므로 에러는 [Firmware](https://github.com/PX4/Firmware) 저장소에서 수정해야만 합니다. 문서는 Firmware 디렉토리의 루트에서 다음 명령을 실행해서 생성할 수 있습니다 :
```
make module_documentation
```
생성된 파일은 `modules` 디렉토리에 들어갑니다.

## 카테고리
- [Command](modules_command.md)
- [Communication](modules_communication.md)
- [Driver](modules_driver.md)
- [Estimator](modules_estimator.md)
- [System](modules_system.md)
