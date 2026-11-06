# Table 1: Proportional-Only Controller Performance Sweep

Plant parameters: $m = 1.0$ kg, $c = 10.0$ Ns/m, $k = 20.0$ N/m. Setpoint $r = 1.0$ m.

| $K_p$ (N/m) | Steady-State Error $e_{ss}$ | Final Value $x_{ss}$ | Overshoot (%) | Rise Time (s) | Settling Time (s) | Peak Force (N) |
|---|---|---|---|---|---|---|
| 10 | 0.6667 m | 0.3333 m | 0.00% | n/a (never reaches 90%) | never settles | 10.00 N |
| 50 | 0.2857 m | 0.7143 m | 0.00% | n/a (never reaches 90%) | never settles | 50.00 N |
| 100 | 0.1667 m | 0.8333 m | 0.00% | 0.184 s | never settles | 100.00 N |
| 300 | 0.0625 m | 0.9375 m | 31.22% | 0.077 s | never settles | 300.00 N |
| 1000 | 0.0196 m | 0.9804 m | 57.56% | 0.036 s | 1.428 s | 1000.00 N |
