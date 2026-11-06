# 📘 Chapter 10: PID Control — Complete Visual Study Notes

> **Reference:** *Feedback Systems: An Introduction for Scientists and Engineers* (Karl Johan Åström & Richard M. Murray), Chapter 10.

---

## 🧭 Overview & Why PID Matters

In real-world industries (chemical processing, oil refining, pulp and paper, robotics, avionics), **more than 95% to 97% of regulatory feedback controllers are PID (or PI) controllers**. 

PID control survives and dominates because it does not require an ultra-precise, complex physical model of the process to get excellent stability and regulation.

---

## 1. 🧩 The Core Concept: What is PID?

A PID controller looks at the **control error** $e(t)$, which is the difference between what you want (**Reference / Setpoint $r$**) and what you currently have (**Process Output $y$**):

$$e(t) = r(t) - y(t)$$

It then creates a corrective actuation command $u(t)$ using three separate lenses:

```
                  ┌──────────────────────────────────────────────┐
                  │              PID CONTROLLER                  │
                  │                                              │
                  │   [ P ]  Present Error:  kp * e(t)           │
   Error e(t)     │                                              │    Control Signal u(t)
 ───────────────► │   [ I ]  Past History:   ki * ∫ e(τ) dτ      │ ────────────────────────►
   (r - y)        │                                              │     (Actuator push)
                  │   [ D ]  Future Trend:   kd * de(t)/dt       │
                  │                                              │
                  └──────────────────────────────────────────────┘
```

### The Universal Analogy: Driving a Car
* **Proportional (P) — "The Present":** You steer harder the further away you are from the lane center.
* **Integral (I) — "The Past":** There is a continuous crosswind blowing you slightly left. P alone won't center you completely because zero error means zero steering force. The Integral accumulates that persistent left drift over time and leans right until you are dead center.
* **Derivative (D) — "The Future":** You see the car swerving toward the guardrail rapidly. You preemptively tap the brakes and counter-steer *before* you crash, damping the violent swing.

---

## 2. 📐 Mathematical Formulation

### 2.1 Standard Ideal Equation (Time Domain)
$$u(t) = k_p e(t) + k_i \int_{0}^{t} e(\tau) \, d\tau + k_d \frac{de(t)}{dt}$$

Often parameterized using time constants ($T_i$: integral time, $T_d$: derivative time):
$$u(t) = k_p \left( e(t) + \frac{1}{T_i} \int_{0}^{t} e(\tau) \, d\tau + T_d \frac{de(t)}{dt} \right)$$
where:
* $k_i = \frac{k_p}{T_i}$
* $k_d = k_p T_d$

### 2.2 Transfer Function (Frequency Domain Laplace Transform)
$$C(s) = k_p + \frac{k_i}{s} + k_d s = k_p \left( 1 + \frac{1}{s T_i} + s T_d \right)$$

---

## 3. 🔍 Deep Dive: The Three Terms

### 3.1 Proportional Action ($P$)
* **Formula:** $u_p(t) = k_p e(t)$
* **Purpose:** Provides the primary corrective muscle proportional to current deviation.
* **Limitation:** Pure proportional control almost always leaves a **steady-state error (droop / offset)**!
  * Why? If $e = 0$, then $u = 0$. But holding an elevator at a certain floor or a car at 60 mph requires a non-zero force ($u > 0$) to fight gravity and drag! Therefore, $e$ must stay non-zero to generate that force.

```
Steady-State Error with P-Control:
Reference r ─────────────────────────────────────────
                        ▲
                        │ Offset / Steady-State Error = 1 / (1 + kp * P(0))
Output y    ────────────┴────────────────────────────
```

---

### 3.2 Integral Action ($I$)
* **Formula:** $u_i(t) = k_i \int_0^t e(\tau) d\tau$
* **Purpose:** **Guarantees zero steady-state error.**
* **Proof from First Principles:**
  Suppose the system reaches a constant steady state where $u(t) \to u_0$ and $e(t) \to e_0$.
  $$u_0 = k_p e_0 + k_i \int_0^t e_0 \, d\tau = k_p e_0 + k_i e_0 \cdot t$$
  As $t \to \infty$, $u_0$ would explode to infinity unless $e_0 = 0$. Thus, $e_0$ **must be identically zero**.
* **Disturbance Rejection Measure:**
  The integrated error for a unit step disturbance input is:
  $$\int_0^\infty e(t) \, dt = \frac{1}{k_i}$$
  *Higher $k_i$ means faster and better disturbance rejection, but too high leads to oscillations and instability.*

---

### 3.3 Derivative Action ($D$)
* **Formula:** $u_d(t) = k_d \frac{de(t)}{dt} = k_p T_d \frac{de(t)}{dt}$
* **Predictive Meaning:** Linear extrapolation ahead in time by $T_d$:
  $$e(t + T_d) \approx e(t) + T_d \frac{de(t)}{dt}$$
  $$u_p + u_d = k_p \cdot e(t + T_d)$$
  The controller acts on where the error **is heading**, adding artificial damping and suppressing overshoots.

```
Error e(t)
  ▲
  │                       Linear prediction: e(t + Td) ≈ e(t) + Td * e'(t)
  │                              /
  │                      *◄─────'
  │                     /│
  │         e(t) ──────* │
  │                   /  │
  │                  /   │
  │                 /    │
  └────────────────┼─────┼────────► Time t
                   t   t + Td
```

---

## 4. ⚙️ Designing Simple Controllers for Complex Systems

Many complex processes can be approximated near their operational equilibrium by low-order models:

### 4.1 Pure Integral Control for Low-Speed Processes
If a process is stable and driven slowly, we can approximate its DC gain by $K = P(0)$.
The closed-loop characteristic equation is:
$$s + K k_i = 0 \implies k_i = \frac{1}{T_{\text{cl}} P(0)}$$
where $T_{\text{cl}}$ is the desired closed-loop time constant.

Using Taylor series expansion around $s = 0$:
$$P(s) \approx P(0) + s P'(0)$$
Choosing robust loop shaping gives:
$$k_i = -\frac{1}{2 P'(0)}$$

---

### 4.2 PI Control for First-Order Processes (Pole Placement)
For a first-order process:
$$P(s) = \frac{b}{s + a}$$
With PI controller $C(s) = k_p + \frac{k_i}{s}$, the closed-loop denominator is:
$$\text{char}(s) = s(s + a) + b(k_p s + k_i) = s^2 + (a + b k_p)s + b k_i$$

To match a desired 2nd-order response $s^2 + 2\zeta \omega_0 s + \omega_0^2$:
$$k_p = \frac{2\zeta \omega_0 - a}{b}, \quad k_i = \frac{\omega_0^2}{b}$$

---

## 5. 🎛️ PID Tuning Methods

When the exact physics is unknown, empirical tuning methods are used.

```
                      ┌──────────────────────────────────────┐
                      │        PID Tuning Approaches         │
                      └──────────────────┬───────────────────┘
                                         │
                 ┌───────────────────────┴───────────────────────┐
                 ▼                                               ▼
     Step Response Method                            Frequency Response Method
     (Open-Loop Step Test)                           (Closed-Loop Oscillation)
     • Find inflection point                         • Increase kp until continuous
     • Extract slope (a) & delay (τ)                   sustained oscillation (kc, Tc)
     • Ziegler-Nichols Table                         • Or use automated Relay Feedback
```

---

### 5.1 Ziegler–Nichols Step Response Method
Apply an open-loop step input of amplitude $1$. Find the steepest slope and draw the tangent line to the inflection point:

```
Process
Output y
  ▲
  │                                   Tangent line at inflection
  │                                     /
  │                                    /
  │                                   /|
  │                                  / |
  │                                 /  |
  │                                /   |
──┼───────────────────────────────*────┼──────────────────────► Time t
 0│◄─────────── τ ───────────────►│    │
  │       (Apparent Delay)             │
  │                                    a (Intercept factor)
```

$$\text{Slope } R = \frac{a}{\tau}$$

| Controller Type | $k_p$ | $T_i$ | $T_d$ |
| :--- | :--- | :--- | :--- |
| **P** | $1/a$ | — | — |
| **PI** | $0.9/a$ | $3\tau$ | — |
| **PID** | $1.2/a$ | $2\tau$ | $0.5\tau$ |

---

### 5.2 Ziegler–Nichols Frequency Response Method
1. Set $k_i = 0, k_d = 0$.
2. Slowly increase $k_p$ until the system begins **sustained, steady sinusoidal oscillations** (the stability boundary).
3. Record:
   * **$k_c$ (Critical Gain):** the value of $k_p$ at oscillation.
   * **$T_c$ (Critical Period):** the period of one complete oscillation cycle ($T_c = \frac{2\pi}{\omega_c}$).

| Controller Type | $k_p$ | $T_i$ | $T_d$ |
| :--- | :--- | :--- | :--- |
| **P** | $0.5 k_c$ | — | — |
| **PI** | $0.4 k_c$ | $0.8 T_c$ | — |
| **PID** | $0.6 k_c$ | $0.5 T_c$ | $0.125 T_c$ |

---

### 5.3 Automated Tuning via Relay Feedback
Manually driving a machine to the edge of instability with high gain can be dangerous. Åström and Hägglund introduced the **Relay Feedback method**:

```
           +──────+   Error e   +────────+  u (Square wave)  +─────────+
    r ───► |  Σ   | ──────────► |  Relay | ────────────────► | Process | ───┬──► y (Sinusoid)
         - |      |             +────────+                   +─────────+    │
           +──▲───+                                                         │
              │                                                             │
              └─────────────────────────────────────────────────────────────┘
```

* The relay switches output: $u = +d$ when $e > 0$, and $u = -d$ when $e < 0$.
* The output oscillates at a controlled, safe limit-cycle amplitude $a$.
* Using the **Fourier series first harmonic** of the square wave:
  $$k_c = \frac{4d}{\pi a}, \quad T_c = \text{Period of the wave}$$
* Once $k_c$ and $T_c$ are calculated automatically, the system automatically switches to the tuned PID controller!

---

## 6. ⚠️ Real-World Engineering Complications & Fixes

Real-world controllers deviate from theoretical equations due to physics limitations:

---

### 6.1 Problem 1: Integrator Windup
* **What happens:** Every physical actuator saturates (a motor hits max voltage, a valve is 100% open). When a large disturbance or large setpoint change occurs, the error $e(t)$ remains non-zero for a long time. The integral term continues adding up ($\int e \, dt \to \text{huge}$).
* **The Disaster:** Even after the system finally reaches the setpoint ($e = 0$), the integral term is so massive that the valve stays stuck 100% open. The system overshoots violently while the integrator slowly "unwinds".

#### The Solution: Tracking Anti-Windup
Measure the difference between the **commanded output $v$** and the **actual saturated physical output $u$**. Feed this error back into the integrator via gain $k_t$:

```
               ┌───────────────────────┐
               │    Anti-Windup Loop   │
               └───────────┬───────────┘
                           │
       +---------+         │         +-------+       v   +------------+    u
 e ──► | ki / s  | ──(+)───┴───────► |   Σ   | ────────► | Saturation | ──────► Process
       +---------+    ▲              +-------+           |  Actuator  |    │
                      │ -                ▲               +------------+    │
                      │                  │                                 │
                   [ kt ] ◄──────────────┴─── es = u - v ◄─────────────────┘
```
* **When normal ($u = v$):** $e_s = 0$, extra loop does nothing.
* **When saturated ($u \ne v$):** $e_s$ actively discharges the integrator, freezing it at the saturation boundary.

---

### 6.2 Problem 2: Derivative Kick & Setpoint Weighting
* **What happens:** If the setpoint $r$ steps abruptly from $0$ to $10$, the derivative of the error $\frac{d(r-y)}{dt}$ produces an infinite mathematical spike (impulse), which violently slams the actuator!
* **The Solution:** Differentiate **only the negative output $-y$**, not the setpoint $r$.
* **Generalized 2-DOF PID Equation:**
  $$u(t) = k_p \left( \beta r(t) - y(t) \right) + k_i \int_0^t (r(\tau) - y(\tau)) \, d\tau + k_d \left( \gamma \frac{dr}{dt} - \frac{dy}{dt} \right)$$
  * Typically, $\beta \in [0, 1]$ (reduces setpoint overshoot).
  * Typically, $\gamma = 0$ (completely eliminates derivative kick from setpoint steps).

---

### 6.3 Problem 3: High-Frequency Sensor Noise
* **What happens:** Sensor signals have small, rapid electrical noise ($\sin(\omega t)$ with large $\omega$). The derivative $\frac{d}{dt} \sin(\omega t) = \omega \cos(\omega t)$ amplifies this noise by $\omega$, causing the actuator to jitter, overheat, and fail mechanically.
* **The Solution:** Add a first-order low-pass filter to the derivative term:
  $$D(s) = \frac{k_d s}{1 + s T_f}$$
  where $T_f = \frac{k_d}{k_p N} = \frac{T_d}{N}$, with $N \in [2, 20]$ (typically $N \approx 10$).

---

## 7. 💻 Practical Computer / Digital Implementation

In software (C/C++, Python, PLC Ladder, microcontrollers), differential equations become difference equations running at sample period $h$:

### Pseudocode Implementation (Anti-Windup + Filtered Derivative + Setpoint Weighting)

```c
// Precomputed Coefficients
bi = ki * h;
ad = Tf / (Tf + h);
bd = kd / (Tf + h);
br = h / Tt;

// Periodic loop executed every sample interval h
while (running) {
    r = read_analog_input(channel_setpoint);
    y = read_analog_input(channel_process_variable);

    // 1. Proportional term with setpoint weighting (beta)
    P = kp * (beta * r - y);

    // 2. Filtered Derivative term on process variable only (gamma = 0)
    D = ad * D - bd * (y - y_old);

    // 3. Controller output before saturation
    v = P + I + D;

    // 4. Actuator Saturation
    u = saturate(v, u_min, u_max);
    write_analog_output(channel_actuator, u);

    // 5. Update Integrator with Anti-Windup tracking term
    I = I + bi * (r - y) + br * (u - v);

    // 6. Save history for next sample
    y_old = y;

    sleep_until_next_tick(h);
}
```

---

## 8. 🧠 Quick Summary Cheat-Sheet

| Term | Domain | Primary Benefit | Potential Danger | Practical Fix |
| :--- | :--- | :--- | :--- | :--- |
| **P** | Present ($e$) | Fast response, closes error gap | Cannot eliminate steady-state offset alone | Add Integral term ($I$) |
| **I** | Past ($\int e$) | Guarantees **zero steady-state error** | Windup (overshoot) when actuator saturates | Anti-windup tracking loop ($k_t$) |
| **D** | Future ($\frac{de}{dt}$) | Adds damping, prevents overshoot | Amplifies high-frequency noise & derivative kick | Low-pass filter ($T_f$) + derive $-y$ only ($\gamma = 0$) |