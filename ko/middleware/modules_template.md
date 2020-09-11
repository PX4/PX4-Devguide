# 모듈 참고: 서식

## 모듈

Source: [templates/template_module](https://github.com/PX4/Firmware/tree/master/src/templates/template_module)

### 설명

제공하는 모듈의 기능을 설명하는 절입니다.

start/stop/status 기능을 지닌 백그라운드 작업 실행 모듈의 서식입니다.

### 구현

이 모듈의 고수준 구현체를 설명하는 절입니다.

### 예제

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

소스 코드: [examples/work_item](https://github.com/PX4/Firmware/tree/master/src/examples/work_item)

### 설명

작업 큐에서 실행하는단순 모듈 예제입니다.

### 사용법 {#work_item_example_usage}

    work_item_example <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info