# 모듈 참고: 시뮬레이션

## sih

Source: [modules/sih](https://github.com/PX4/Firmware/tree/master/src/modules/sih)

### 설명

이 모듈은 쿼드 콥터의 자동 조종 장치 하드웨어를 완벽히 지원하는 모의 시험 환경을 제공합니다.

이 모의 시험 환경에서는 믹서에서 전달하는 엑추에이터 pwm 신호 "actuator_outputs"를 주기적으로 수신합니다.

또한 모의 시험 환경에서는 루프 로직 상에서 상태 추정자가 개입하게 하는 실제 잡음 신호로 깨진 센서 신호를 보내기도 합니다.

### 구현

모의 시험 환경에는 행렬 수식으로 모션 방정식을 구현했습니다. 자세 표현시 4차원 행렬을 활용합니다. 통합을 위해서는 forward euler가 사용됩니다. 대부분의 변수들은 스택 오버플로우를 방지하기 위해 .hpp에 선언되었습니다.

### 사용법 {#sih_usage}

    sih <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info