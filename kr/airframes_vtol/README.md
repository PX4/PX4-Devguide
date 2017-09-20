# VTOL Airframes

[PX4 Flight Stack](../concept/flight_stack.md)에서 대부분의 VTOL 설정을 제공합니다 :

  * Tailsitters (2개와 4개로 구성되는 rotor로 X와 + 설정지원)
  * Tiltrotors (Firefly Y6)
  * 표준 plane VTOL (비행체에 쿼드로터가 추가된 형태)

VTOL 코드베이스는 다른 모든 비행체에 대해서 동일한 코드베이스를 사용합니다. 특별히 transition되는 부분에 대해서 제어 로직을 추가하기만 하면 됩니다.

> **Note** 이런 모든 VTOL 설정으로 활발하게 실험비행을 진행하고 있으며 현재 사용가능한 상태입니다. airspeed 센서를 시스템에 반드시 부착해서 transition을 안전하게 수행할 수 있도록 합니다.


## 핵심 설정 Parameters

새로운 비행체 설정을 생성할 때는, 여기서 다루는 설정 parameter들이 올바르게 설정되도록 해야 합니다.

  * `VT_FW_PERM_STAB` 시스템은 hover 모드에서 항상 attitude stabilization을 사용. 만약 이 파라미터가 1로 설정되면, plane 모드도 기본으로 attitude stabilization가 된다. 만약 0으로 설정하면, 기본으로 완전한 manual flight가 된다.
  * `VT_ARSP_TRANS` m/s 단위의 airspeed를 뜻하며 forward flight로 transition이 된다. 너무 낮은 값으로 설정하면 transition 동안에 stall이 발생할 수 있다.
  * `RC_MAP_TRANS_SW` 비행하기 전에 RC 스위치에 할당해야 한다. 멀티로터와 고정익 모드가 잘 동작하는지 체크할 수 있다. (비행중에 2가지 제어 모드를 스위치로 수동으로 설정할 수 있음)

## Tailsitter

[build log](https://docs.px4.io/en/frames_vtol/vtol_tailsitter_caipiroshka_pixracer.html)에서 상세한 정보를 확인하세요.

{% youtube %}https://www.youtube.com/watch?v=acG0aTuf3f8&vq=hd720{% endyoutube %}

## Tiltrotor

[build log](https://pixhawk.org/platforms/vtol/birdseyeview_firefly)에서 설정 및 구성방법에 대한 정보를 얻을 수 있습니다.

{% youtube %}https://www.youtube.com/watch?v=Vsgh5XnF44Y&vq=hd720{% endyoutube %}

## 표준 Plane VTOL

[build log](https://pixhawk.org/platforms/vtol/fun_cub_quad_vtol) 아래와 같은 결과를 얻기 위해 구성할 수 있는 방법에 대한 정보를 얻을 수 있습니다.

{% youtube %}https://www.youtube.com/watch?v=4K8yaa6A0ks&vq=hd720{% endyoutube %}

{% youtube %}https://www.youtube.com/watch?v=7tGXkW6d3sA&vq=hd720{% endyoutube %}
