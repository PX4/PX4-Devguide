# 모듈 레퍼런스: 템플릿

## 모듈

Source: [templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module)

### 설명

제공하는 모듈 기능을 설명하는 섹션입니다.

시작/종료/상태 기능을 가진 백그라운드에서 수행되는 작업을 수행하는 모듈을 위한 템플릿입니다.

### 구현

이 모듈을 구현하는 방법을 설명하는 섹션

### Examples

CLI 사용 예:

    module start -f -p 42
    

### 사용법 {#module_usage}

    module <command> [arguments...]
     Commands:
       start
         [-f]        Optional example flag
         [-p <val>]  Optional example parameter
                     default: 0
    
       stop
    
       status        print status info