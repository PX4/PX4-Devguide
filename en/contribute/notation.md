# Terminology/Notation

The following terms, symbols, and decorators are used in text and diagrams throughout this guide.

## Acronyms

Acronym | Expansion
--- | ---
AOA | Angle Of Attack. Also named *alpha*.
AOS | Angle Of Sideslip. Also named *beta*.
FRD | Coordinate system where the X-axis is pointing towards the Front of the vehicle, the Y-axis is pointing Right and the Z-axis is pointing Down, completing the right-hand rule.
FW | Fixed-Wing.
MC | MultiCopter.
MPC or MCPC | MultiCopter Position Controller. MPC is also used for Model Predictive Control.
NED | Coordinate system where the X-axis is pointing towards the true North, the Y-axis is pointing East and the Z-axis is pointing Down, completing the right-hand rule.
PID | Controller with Proportional, Integral and Derivative actions.


## Symbols

Symbol | Description
--- | ---
$$\boldsymbol{\mathrm{a}}$$ | Acceleration vector. $$\boldsymbol{\mathrm{a}} = \boldsymbol{\mathrm{\dot{v}}} = \boldsymbol{\mathrm{\ddot{r}}} = [a_x \quad a_y \quad a_z]^T$$.
$$\alpha$$ | Angle of attack (AOA).
$$AR$$ | Aspect ratio. $$AR = b^2/S$$.
$$b$$ | Wing span (from tip to tip).
$$\beta$$ | Angle of sideslip (AOS).
$$\bar{c}$$ | Mean aerodynamic chord (mac).
$$c$$ | Wing chord length.
$$\delta_{a,e,r}$$ | Aerodynamic control surface angular deflection. Subscripts $$a$$, $$e$$ and $$r$$ stand for *aileron*, *elevator* and *rudder*, respectively. A positive deflection generates a negative moment.
$$\Psi$$ | Attitude vector. $$\Psi = [\phi \quad \theta \quad \psi]^T$$.
$$\phi$$ | Roll euler angle. Also named *Bank angle* in aviation.
$$\psi$$ | Yaw euler angle. Also named *Heading*.
$$\theta$$ | Pitch euler angle.
$$\boldsymbol{\mathrm{F}}$$ | Force vector. $$\boldsymbol{\mathrm{F}} = [X \quad Y \quad Z]^T$$.
$$\boldsymbol{\mathrm{F}}_{Aero}^w$$ | Aerodynamic forces in wind frame. *Lift* $$L$$, *drag* $$D$$ and *cross-wind force* $$C$$. $$\boldsymbol{\mathrm{F}}_{Aero}^w = [-D \quad -C \quad -L]^T$$.
$$\boldsymbol{\mathrm{F}}_T^b$$ | Thrust force in body frame. $$\boldsymbol{\mathrm{F}}_T^b = [X_T^b \quad Y_T^b \quad Z_T^b]^T$$.
$$\boldsymbol{\mathrm{g}}$$ | Gravity vector.
$$\boldsymbol{\mathrm{M}}_{Aero}^b$$ | Body aerodynamic moments. $$\boldsymbol{\mathrm{M}}_{Aero}^b = [\ell \quad m \quad n]^T$$.
$$\boldsymbol{\mathrm{M}}_T^b$$ | Body thrust moments. $$\boldsymbol{\mathrm{M}}_T^b = [\ell_T \quad m_T \quad n_T]^T$$.
$$M$$ | Mach number. Can be neglected for scale aircrafts.
$$\boldsymbol{\mathrm{\tilde{q}}}$$ | Hamiltonian attitude quaternion. $$\boldsymbol{\mathrm{\tilde{q}}} = (q_0, q_1, q_2, q_3) = (q_0, \boldsymbol{\mathrm{q}})$$.<br>A vector in the local NED frame $$\ell$$ can be represented in the body frame $$b$$ using $$\boldsymbol{\mathrm{\tilde{v}}}^b = \boldsymbol{\mathrm{\tilde{q}}} \, \boldsymbol{\mathrm{\tilde{v}}}^\ell \, \boldsymbol{\mathrm{\tilde{q}}}^*$$ (or $$\boldsymbol{\mathrm{\tilde{q}}}^{-1}$$ instead of $$\boldsymbol{\mathrm{\tilde{q}}}^*$$ if $$\boldsymbol{\mathrm{\tilde{q}}}$$ is not unitary).  $$\boldsymbol{\mathrm{\tilde{v}}}$$ represents a *quaternionized* vector: $$\boldsymbol{\mathrm{\tilde{v}}} = (0,\boldsymbol{\mathrm{v}})$$.
$$\boldsymbol{\mathrm{r}}$$ | Position vector $$\boldsymbol{\mathrm{r}} = [x \quad y \quad z]^{T}$$.
$$\boldsymbol{\mathrm{R}}_a^b$$ | Rotation matrix. Rotates a vector from frame $a$ to frame $b$. $$\boldsymbol{\mathrm{v}}^b = \boldsymbol{\mathrm{R}}_a^b \boldsymbol{\mathrm{v}}^a$$.
$$\Lambda$$ | Leading-edge sweep angle.
$$\lambda$$ | Taper ratio $$\lambda = c_{tip}/c_{root}$$.
$$\boldsymbol{\mathrm{v}}$$ | Velocity vector. $$\boldsymbol{\mathrm{v}} = \boldsymbol{\mathrm{\dot{r}}} = [v_x \quad v_y \quad v_z]^T$$.
$$\boldsymbol{\mathrm{v}}^\ell$$ | Velocity vector in local frame. $$\boldsymbol{\mathrm{v}}^\ell = \boldsymbol{\mathrm{v}}_w^\ell + \boldsymbol{\mathrm{w}}^\ell$$.
$$\boldsymbol{\mathrm{v}}_w^b$$ | Relative airspeed velocity vector in body frame. $$\boldsymbol{\mathrm{v}}_w^b = [u \quad v \quad w]^T$$.
$$\boldsymbol{\mathrm{w}}^\ell$$ | Wind velocity vector in local frame. $$\boldsymbol{\mathrm{w}}^\ell = [w_N \quad w_E \quad w_D]^T$$. Usually $$w_D$$ is assumed to be null.
$$\boldsymbol{\omega}^b$$ | Body rates vector. $$\boldsymbol{\omega}^b = [p \quad q \quad r]^T$$.
$$\boldsymbol{\mathrm{x}}$$ | General state vector.


## Decorators

Decorator | Description
--- | ---
$$()^*$$ | Complex conjugate.
$$\dot{()}$$ | Time derivative.
$$()^b$$ | Resolved in the body FRD frame.
$$()^\ell$$ | Resolved in the local NED frame.
$$()^w$$ | Resolved in the wind frame.
$$\hat{()}$$ | Estimate.
$$()^{-1}$$ | Matrix inverse.
$$()^T$$ | Matrix transpose.
