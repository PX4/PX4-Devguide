# PWM_limit 状态机

[PWM_limit 状态机] 根据锁定（pre-armed）和解锁（armed）状态作为输入量控制飞控的 PWM 输出， 并且会在解锁指令发出后、飞机油门增加之前引入一个延时。

## 总览

**输入**

- armed: asserted to enable dangerous behaviors such as spinning propellers
- pre-armed: asserted to enable benign behaviors such as moving control surfaces 
    - this input overrides the current state
    - assertion of pre-armed immediately forces behavior of state ON, regardless of current state ** deassertion of pre-armed reverts behavior to current state

**States**

- INIT and OFF 
    - pwm outputs set to disarmed values.
- RAMP 
    - pwm ouputs ramp from disarmed values to min values.
- ON 
    - pwm outputs set according to control values.

## State Transition Diagram

![](../../assets/diagrams/pwm_limit_state_diagram.png)