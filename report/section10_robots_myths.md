# Section 10: Real-World Robot Case Studies & Engineering Myths

## Part 1: Real-World Robotics Case Studies

PID controllers govern over 90% of industrial control loops and modern autonomous machines. Below are five real-world robotic systems, detailing their physical error signals, actuators, and practical gain tuning objectives.

---

### 1. Automotive Cruise Control & Speed Regulation
- **Physical Error Signal:** $e(t) = v_{\text{target}} - v_{\text{measured}}$, where velocity is measured by wheel speed Hall-effect sensors or transmission tachometers.
- **Actuator:** Throttle body butterfly valve servo angle or electronic fuel injection pulse width (modulating engine output torque).
- **Control Architecture & Tuning:**
  - Primarily a **PI controller**. 
  - Road friction and aerodynamic drag create a continuous resisting force that varies with speed and road incline. Proportional action alone produces steady-state speed droop on uphill climbs.
  - Integral action ($K_i$) builds up throttle command to climb hills without speed loss.
  - Derivative action is typically avoided or heavily filtered because vehicle speed sensors exhibit road-bump vibration, and massive vehicle inertia naturally filters out high-frequency speed fluctuations.

---

### 2. Quadcopter Drone Attitude & Rate Stabilization
- **Physical Error Signal:** Cascaded two-loop structure:
  - *Outer loop:* Attitude orientation error $e_\theta(t) = \theta_{\text{desired}} - \theta_{\text{estimated}}$ from IMU Kalman filtering (accelerometer + gyro fusion).
  - *Inner loop:* Angular velocity error $e_\omega(t) = \omega_{\text{desired}} - \omega_{\text{gyro}}$ measured directly by high-rate MEMS gyroscopes.
- **Actuator:** Electronic Speed Controllers (ESCs) commanding Pulse Width Modulation (PWM) duty cycles to four brushless DC motors, altering differential thrust.
- **Control Architecture & Tuning:**
  - High-gain **PD / PID control** on angular rate.
  - D action provides vital aerodynamic braking against rapid turbulent gusts.
  - Because drone frames vibrate from motor spinning, derivative filtering ($\tau \approx 10\text{--}20\text{ ms}$) is strictly mandatory to prevent ESC motor overheating and battery drain.

---

### 3. Robotic Arm Joint Position Control
- **Physical Error Signal:** $e(t) = \theta_{\text{target}} - \theta_{\text{actual}}$, measured by high-resolution optical shaft encoders or magnetic resolvers on joint gearboxes.
- **Actuator:** Brushless DC motor current / torque vector drives coupled through harmonic drive reduction gears.
- **Control Architecture & Tuning:**
  - **PID with Gravity Feedforward**: Holding an arm outstretched against gravity requires continuous torque (identical to the 20 N spring holding force in our benchmark system).
  - While $K_i$ can learn the gravity torque, adding feedforward ($u_{\text{ff}} = m g l \cos\theta$) prevents the arm from dipping before the integrator accumulates.
  - High derivative gain ($K_d$) quenches mechanical resonances and structural flexing during high-speed pick-and-place trajectories.

---

### 4. Autonomous Line-Following Mobile Robot
- **Physical Error Signal:** Lateral tracking offset $e(t) = x_{\text{line}} - x_{\text{chassis}}$, measured by an array of infrared reflectance phototransistors sensing the contrast between floor tape and background.
- **Actuator:** Pulse-width-modulated H-bridge motor drivers controlling the differential RPM of left and right drive wheels.
- **Control Architecture & Tuning:**
  - Fast **PD controller**.
  - Proportional action steers wheels proportional to lateral deviation.
  - Derivative action dampens the turning rate as the robot realigns with the tape, preventing the vehicle from fishtailing or oscillating off the line at sharp corners.
  - Integral action is intentionally kept near zero to avoid path hunting and integrator windup during straightaways.

---

### 5. Precision DC Motor Speed & Current Servo
- **Physical Error Signal:** Shaft rotational speed error $e_\omega(t) = \omega_{\text{target}} - \omega_{\text{encoder}}$.
- **Actuator:** Full-bridge MOSFET power inverter applying voltage pulses to motor armature windings.
- **Control Architecture & Tuning:**
  - Dual nested loops: an ultra-fast inner PI current/torque loop (~10 kHz) nested inside an outer PI/PID speed loop (~1 kHz).
  - Integral action compensates for mechanical load friction and back-EMF variations, maintaining constant spindle speed under changing cutting tool resistance.

---

## Part 2: Five Prevalent PID Myths Debunked by Empirical Data

---

### Myth 1: "More gain is always better — just crank up $K_p$ to make the system fast."
- **The Wrong Belief:** If a controller responds too slowly, increasing proportional gain $K_p$ indefinitely will yield an infinitely fast, zero-delay response.
- **The Physical Reality:** In physical systems, increasing $K_p$ raises the effective stiffness without increasing damping. As proven by closed-loop damping ratio $\zeta = \frac{c}{2\sqrt{m(k + K_p)}}$, higher $K_p$ directly reduces damping, driving the system into violent resonance and oscillatory overshoot.
- **Direct Code Evidence:** Look at **Figure 5 (F05)**. Raising $K_p$ from 100 to 1000 N/m increases overshoot from 0% to 57.6%. Furthermore, it demands a massive initial impulse force of $1000\text{ N}$ at $t=0$, instantly exceeding real motor force limits and damaging gear teeth.

---

### Myth 2: "The Integral term fixes everything — just add $K_i$ whenever you have an error."
- **The Wrong Belief:** Since the integral term eliminates steady-state error, setting a high $K_i$ guarantees perfect target tracking under all conditions.
- **The Physical Reality:** Pure integration adds $90^\circ$ of phase lag to the feedback loop. When actuators saturate at physical force or torque limits, the error continues to accumulate unchecked in the software accumulator ("integrator windup"). When the machine finally reaches the setpoint, the overloaded integrator cannot dump its stored charge immediately, forcing the machine to overshoot wildly.
- **Direct Code Evidence:** Look at **Figure 11 (F11)**. Under a 30 N motor ceiling, unclamped integral accumulation causes a $17.6\%$ overshoot and takes $2.98\text{ s}$ to settle. Only by introducing conditional clamping anti-windup is overshoot suppressed to $0.0\%$ and settling accelerated to $1.06\text{ s}$.

---

### Myth 3: "Derivative action predicts the future perfectly and provides ideal anticipation."
- **The Wrong Belief:** Because the derivative term is mathematically proportional to the error slope ($K_d \frac{de}{dt}$), it acts as an ideal predictive telescope that foresees where the system is heading.
- **The Physical Reality:** Physical sensors do not output smooth analytical functions; they produce discrete measurements corrupted by high-frequency electrical noise, quantization, and mechanical vibration. Discrete differentiation of noise $\sigma$ across sample interval $\Delta t$ scales noise amplitude by $\frac{\sqrt{2}\sigma}{\Delta t}$.
- **Direct Code Evidence:** Look at **Figure 12 (F12)**. With just $\pm 1\text{ mm}$ of sensor noise, raw derivative action produces $43.4\text{ N}$ RMS force chatter across the motor—completely consuming the actuator's 30 N authority with pure acoustic buzzing and heat. Derivative action is unusable in physical hardware without a low-pass filter ($\tau = 0.02\text{--}0.05\text{ s}$).

---

### Myth 4: "One set of tuned gains works across all operating conditions."
- **The Wrong Belief:** Once a PID controller is tuned on a test bench, those gains remain optimal regardless of system payload, wear, or environmental conditions.
- **The Physical Reality:** Linear PID assumes fixed physical plant parameters ($m, c, k$). In reality, robot manipulators pick up payloads, mobile robots carry variable cargo, and mechanisms heat up. Doubling the effective mass shifts natural frequency $\omega_n = \sqrt{k/m}$ and damping $\zeta = c / (2\sqrt{mk})$, altering closed-loop stability margins.
- **Direct Code Evidence:** Look at the **Mass Sensitivity Test** (`playground.py --mass 2.0`). When payload mass doubles from $1.0\text{ kg} \to 2.0\text{ kg}$ under the identical baseline gains ($300/300/30$), overshoot jumps from $4.27\%$ to $6.92\%$ and initial acceleration drops by half.

---

### Myth 5: "Ziegler-Nichols formulas provide the final, optimal control gains."
- **The Wrong Belief:** Entering $K_u$ and $T_u$ into the classical Ziegler-Nichols lookup table produces a production-ready, professionally tuned PID controller.
- **The Physical Reality:** The classical 1942 Ziegler-Nichols tuning recipe was explicitly designed for quarter-decay ratio response in process control (chemical plants where fast disturbance rejection was valued over smooth positioning). In motion control and robotics, quarter-amplitude decay corresponds to an unacceptable ~50% overshoot, which would slam robotic linkages against mechanical hardstops.
- **Direct Code Evidence:** Look at **Figure 16 (F16)** and **Table 5**. Applying classic ZN PID to the 3-lag plant produces a violent **$52.44\%$ overshoot** and sustained ringing for $9.9\text{ s}$. Tailored engineering rules (such as Tyreus-Luyben or our improved tuning: $K_p=3.60, K_i=0.80, K_d=3.20$) are mandatory to tame overshoot down to $3.10\%$ while achieving clean settling under $10\text{ s}$.
