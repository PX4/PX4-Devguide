# 용어

다음 용어, 기호, 장식 문자는 이 안내서 전체의 텍스트와 다이어그램에 활용합니다.

## 표기

- 굵은 서체 변형은 벡터, 행렬을 나타내며, 굵지 않은 서체는 스칼라 배열을 나타냅니다. 
- 각 변수의 기본 프레임은 로컬 프레임 $$\ell$$ 입니다. 우측 [위첨자](#superscripts)는 좌표 프레임을 나타냅니다. 우측 위첨자가 없으면 $$\ell$$ 기본 프레임임을 가정합니다. 회전 행렬일 경우 예외입니다. 우측 아래첨자로 현재 프레임을 나타내며 우측 위첨자로 대상 프레임을 나타냅니다.
- 변수와 아래 첨자는 동일한 문자를 쓸 수 있으나, 늘 다른 의미를 지닙니다.

## 약어

| 축약어         | Expansion                                                                                                                                                                      |
| ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| AOA         | Angle Of Attack. Also named *alpha*.                                                                                                                                           |
| AOS         | Angle Of Sideslip. Also named *beta*.                                                                                                                                          |
| FRD         | Coordinate system where the X-axis is pointing towards the Front of the vehicle, the Y-axis is pointing Right and the Z-axis is pointing Down, completing the right-hand rule. |
| FW          | Fixed-Wing.                                                                                                                                                                    |
| MC          | MultiCopter.                                                                                                                                                                   |
| MPC or MCPC | MultiCopter Position Controller. MPC is also used for Model Predictive Control.                                                                                                |
| NED         | Coordinate system where the X-axis is pointing towards the true North, the Y-axis is pointing East and the Z-axis is pointing Down, completing the right-hand rule.            |
| PID         | Controller with Proportional, Integral and Derivative actions.                                                                                                                 |

## 기호

| 변수                                     | 설명                                                                                                                                                                                                                                                                                                                                                                                                     |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| $$x,y,z$$                              | x, y, z 각 좌표를 따르는 변환                                                                                                                                                                                                                                                                                                                                                                                   |
| $$\boldsymbol{\mathrm{r}}$$          | $$\boldsymbol{\mathrm{r}} = [x \quad y \quad z]^{T}$$ 벡터 위치                                                                                                                                                                                                                                                                                                                                        |
| $$\boldsymbol{\mathrm{v}}$$          | $$\boldsymbol{\mathrm{v}} = \boldsymbol{\mathrm{\dot{r}}}$$ 속도 벡터                                                                                                                                                                                                                                                                                                                                 |
| $$\boldsymbol{\mathrm{a}}$$          | $$\boldsymbol{\mathrm{a}} = \boldsymbol{\mathrm{\dot{v}}} = \boldsymbol{\mathrm{\ddot{r}}}$$ 가속 벡터                                                                                                                                                                                                                                                                                             |
| $$\alpha$$                            | 공격 각도 (AOA).                                                                                                                                                                                                                                                                                                                                                                                           |
| $$b$$                                  | 주익 길이 (끝에서 끝까지)                                                                                                                                                                                                                                                                                                                                                                                        |
| $$S$$                                  | 주익 넓이                                                                                                                                                                                                                                                                                                                                                                                                  |
| $$AR$$                                 | 종횡비 $$AR = b^2/S$$                                                                                                                                                                                                                                                                                                                                                                                     |
| $$\beta$$                             | 측면 경사도 (AOS)                                                                                                                                                                                                                                                                                                                                                                                           |
| $$c$$                                  | 주익현 길이                                                                                                                                                                                                                                                                                                                                                                                                 |
| $$\delta$$                            | 기체 역학 제어 표면 이탈각(손실각). 손실 값이 양인 경우 부모멘트를 생성합니다.                                                                                                                                                                                                                                                                                                                                                         |
| $$\phi,\theta,\psi$$                | 오일러 각. roll(=Bank), pitch, yaw(=Heading).                                                                                                                                                                                                                                                                                                                                                              |
| $$\Psi$$                              | 고도 벡터. $$\Psi = [\phi \quad \theta \quad \psi]^T$$.                                                                                                                                                                                                                                                                                                                                              |
| $$X,Y,Z$$                              | x, y, z 축 방향의 힘                                                                                                                                                                                                                                                                                                                                                                                        |
| $$\boldsymbol{\mathrm{F}}$$          | $$\boldsymbol{\mathrm{F}}= [X \quad Y \quad Z]^T$$ 힘의 벡터                                                                                                                                                                                                                                                                                                                                           |
| $$D$$                                  | 견인력                                                                                                                                                                                                                                                                                                                                                                                                    |
| $$C$$                                  | 측풍력                                                                                                                                                                                                                                                                                                                                                                                                    |
| $$L$$                                  | 양력                                                                                                                                                                                                                                                                                                                                                                                                     |
| $$g$$                                  | 중력                                                                                                                                                                                                                                                                                                                                                                                                     |
| $$l,m,n$$                              | x, y, z 좌표 축 주변의 모멘트                                                                                                                                                                                                                                                                                                                                                                                   |
| $$\boldsymbol{\mathrm{M}}$$          | $$\boldsymbol{\mathrm{M}} = [l \quad m \quad n]^T$$ 모멘트 벡터                                                                                                                                                                                                                                                                                                                                         |
| $$M$$                                  | 마하 계수. 항공기 크기에 따라 무시할 수 있습니다.                                                                                                                                                                                                                                                                                                                                                                          |
| $$\boldsymbol{\mathrm{q}}$$          | 4원수 벡터 영역.                                                                                                                                                                                                                                                                                                                                                                                             |
| $$\boldsymbol{\mathrm{\tilde{q}}}$$ | 해밀터니언 고도 4원수 $$\boldsymbol{\mathrm{\tilde{q}}} = (q_0, q_1, q_2, q_3) = (q_0, \boldsymbol{\mathrm{q}})$$.  
$$\boldsymbol{\mathrm{\tilde{q}}}$$ 로컬 프레임 $$\ell$$에 상대적인 고도를 설명합니다. 바디 프레임에 벡터가 주어졌을 때 로컬 프레임을 나타낸다면 다음 수식을 활용할 수 있습니다: $$\boldsymbol{\mathrm{\tilde{v}}}$$ represents a *quaternionized* vector: $$\boldsymbol{\mathrm{\tilde{v}}} = (0,\boldsymbol{\mathrm{v}})$$. |
| $$\boldsymbol{\mathrm{R}}_\ell^b$$  | Rotation matrix. Rotates a vector from frame $$\ell$$ to frame $$b$$. $$\boldsymbol{\mathrm{v}}^b = \boldsymbol{\mathrm{R}}_\ell^b \boldsymbol{\mathrm{v}}^\ell$$.                                                                                                                                                                                                                            |
| $$\Lambda$$                           | Leading-edge sweep angle.                                                                                                                                                                                                                                                                                                                                                                              |
| $$\lambda$$                           | Taper ratio $$\lambda = c_{tip}/c_{root}$$.                                                                                                                                                                                                                                                                                                                                                         |
| $$w$$                                  | Wind velocity.                                                                                                                                                                                                                                                                                                                                                                                         |
| $$p,q,r$$                              | Angular rates around body axis x,y and z.                                                                                                                                                                                                                                                                                                                                                              |
| $$\boldsymbol{\omega}^b$$            | Angular rate vector in body frame $$\boldsymbol{\omega}^b = [p \quad q \quad r]^T$$.                                                                                                                                                                                                                                                                                                               |
| $$\boldsymbol{\mathrm{x}}$$          | General state vector.                                                                                                                                                                                                                                                                                                                                                                                  |

### 아래첨자 / 인덱스

| 아래첨자 / 인덱스 | 설명                       |
| ---------- | ------------------------ |
| $$a$$      | 보조익                      |
| $$e$$      | 승강타                      |
| $$r$$      | 방향타                      |
| $$Aero$$   | 기체역학                     |
| $$T$$      | 강제 추력                    |
| $$w$$      | 상대 공속                    |
| $$x,y,z$$  | x, y, z 축에 따른 벡터 요소      |
| $$N,E,D$$  | 북, 동, 하 글로벌 방위에 따른 벡터 요소 |

### 위첨자 / 인덱스 {#superscripts}

| 위첨자 / 인덱스 | 설명                  |
| --------- | ------------------- |
| $$\ell$$ | 로컬 프레임 PX4 기본 상대 변수 |
| $$b$$     | 바디 프레임              |
| $$w$$     | 윈드 프레임              |

## Decorators

| Decorator       | Description        |
| --------------- | ------------------ |
| $$()^*$$        | Complex conjugate. |
| $$\dot{()}$$   | Time derivative.   |
| $$\hat{()}$$   | Estimate.          |
| $$\bar{()}$$   | Mean.              |
| $$()^{-1}$$     | Matrix inverse.    |
| $$()^T$$        | Matrix transpose.  |
| $$\tilde{()}$$ | Quaternion.        |