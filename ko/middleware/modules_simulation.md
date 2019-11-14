# 모듈 참고: 시뮬레이션

## sih

Source: [modules/sih](https://github.com/PX4/Firmware/tree/master/src/modules/sih)

### 설명

이 모듈은 쿼드 콥터의 자동 조종 장치 하드웨어를 완벽히 지원하는 시뮬레이터를 제공합니다.

이 시뮬레이터는 믹서가 발생하는 엑추에이터 pwm 신호인 "actuator_outputs"를 구독합니다.

이 시물레이터는 상태 추정기를 통합하기 위해 현실적인 노이즈에 변질된 센서 신호들을 퍼블리시합니다.

### 구현

이 시뮬레이터는 행렬 계산을 이용해 모션 방정식을 구현했습니다. 쿼터니언 표현은 자세에 사용됩니다. 통합을 위해서는 forward euler가 사용됩니다. 대부분의 변수들은 스택 오버플로우를 방지하기 위해 .hpp에 선언되었습니다.

### 사용법 {#sih_usage}

    sih <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info