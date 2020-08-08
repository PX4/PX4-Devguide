# 모듈 참고: 서식

## 모듈

소스 코드: [templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module)

### 설명

제공하는 모듈의 기능을 설명하는 절입니다.

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
    

## work_item_example

Source: [examples/work_item](https://github.com/PX4/Firmware/tree/master/src/examples/work_item)

### Description

Example of a simple module running out of a work queue.

### Usage {#work_item_example_usage}

    work_item_example <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info