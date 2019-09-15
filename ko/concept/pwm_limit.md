# PWM_limit 상태 머신

[PWM_limit State Machine] Controls PWM outputs as a function of pre-armed and armed inputs. Provides a delay between assertion of "armed" and a ramp-up of throttle on assertion of the armed signal.

## Quick Summary

**입력**

- armed: 프로펠러는 돌리는 것과 같이 위험한 행동을 하도록 활성화 된 것
- pre-armed: 제어 표면을 움직이기 시작하도록 활성화 된 것 
    - 이 입력은 현재의 상태보다 더 우선시 된다.
    - pre-armed의 주장은 현재의 상태와 상관없이 즉시 ON 상태를 강요한다.
    - pre-armed의 철회는 현재 상태로 되돌아가도록 한다.

**상태**

- INIT 과 OFF 
    - pwm outputs set to disarmed values.
- RAMP 
    - pwm ouputs ramp from disarmed values to min values.
- ON 
    - pwm outputs set according to control values.

## State Transition Diagram

![](../../assets/diagrams/pwm_limit_state_diagram.png)