"""Sanity test suite verifying physics, math, and PID algorithms against reference benchmarks.

Run with:
    python3 tests/test_sanity.py
"""

import sys
import os
import numpy as np

# Ensure repository root is on Python module search path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from pidlab.plants import MassSpringDamper, ThreeLags
from pidlab.pid import PID
from pidlab.sim import simulate
from pidlab.metrics import calc_metrics, find_peaks_simple


def test_holding_force_settles_at_1m():
    """Verify that applying constant u = 20 N settles at exactly x = 1.0 m."""
    plant = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    dt = 0.001
    for _ in range(5000):  # 5 seconds
        plant.step(u=20.0, dt=dt)
    assert abs(plant.x - 1.0) < 1e-4, f"Holding force 20 N gave x = {plant.x}, expected 1.0"
    print("  [PASS] test_holding_force_settles_at_1m")


def test_p_only_steady_state_errors():
    """Verify P-only steady-state errors match theory k / (Kp + k) within 1%."""
    k_plant = 20.0
    kp_cases = [10.0, 50.0, 100.0, 300.0, 1000.0]

    for kp in kp_cases:
        plant = MassSpringDamper(m=1.0, c=10.0, k=k_plant)
        ctrl = PID(kp=kp, ki=0.0, kd=0.0, dt=0.001)
        sim = simulate(plant, ctrl, setpoint=1.0, T=6.0, dt_sim=0.001)
        metrics = calc_metrics(sim["t"], sim["y"], sim["r"])

        expected_ess = k_plant / (kp + k_plant)
        sim_ess = metrics["steady_state_error"]

        error_diff = abs(sim_ess - expected_ess)
        assert error_diff < 0.01, f"Kp={kp}: sim e_ss={sim_ess:.4f}, expected {expected_ess:.4f}"

        # Overshoot checks for Kp=300 (expect ~31.2%) and Kp=1000 (expect ~57.6%)
        if kp == 300.0:
            assert abs(metrics["overshoot"] - 31.2) < 1.0, f"Kp=300 overshoot {metrics['overshoot']}%"
        elif kp == 1000.0:
            assert abs(metrics["overshoot"] - 57.6) < 1.0, f"Kp=1000 overshoot {metrics['overshoot']}%"

    print("  [PASS] test_p_only_steady_state_errors")


def test_dt_sensitivity():
    """Verify halving dt (1 ms -> 0.5 ms) changes PID overshoot by < 0.1 percentage points."""
    plant1 = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    ctrl1 = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001)
    sim1 = simulate(plant1, ctrl1, setpoint=1.0, T=4.0, dt_sim=0.001)
    os1 = calc_metrics(sim1["t"], sim1["y"], sim1["r"])["overshoot"]

    plant2 = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    ctrl2 = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.0005)
    sim2 = simulate(plant2, ctrl2, setpoint=1.0, T=4.0, dt_sim=0.0005)
    os2 = calc_metrics(sim2["t"], sim2["y"], sim2["r"])["overshoot"]

    diff = abs(os1 - os2)
    assert diff < 0.1, f"Overshoot difference between dt=1ms and 0.5ms is {diff:.4f}%, expected < 0.1%"
    print(f"  [PASS] test_dt_sensitivity (dt=1ms: {os1:.2f}%, dt=0.5ms: {os2:.2f}%, diff={diff:.4f}%)")


def test_unbounded_saturation_invariance():
    """Verify that with u_max = inf, clamping/saturation produces identical output to unbounded."""
    plant1 = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    ctrl1 = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, u_min=-np.inf, u_max=np.inf, anti_windup="clamp")
    sim1 = simulate(plant1, ctrl1, setpoint=1.0, T=3.0, dt_sim=0.001)

    plant2 = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    ctrl2 = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, u_min=-np.inf, u_max=np.inf, anti_windup="none")
    sim2 = simulate(plant2, ctrl2, setpoint=1.0, T=3.0, dt_sim=0.001)

    assert np.allclose(sim1["y"], sim2["y"]), "Saturation logic altered behavior when limits are infinite"
    print("  [PASS] test_unbounded_saturation_invariance")


def test_windup_clamping_effectiveness():
    """Verify anti-windup clamping reduces overshoot under 30 N saturation."""
    # Without anti-windup
    plant_none = MassSpringDamper()
    ctrl_none = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, u_min=-30.0, u_max=30.0, anti_windup="none")
    sim_none = simulate(plant_none, ctrl_none, setpoint=1.0, T=5.0)
    m_none = calc_metrics(sim_none["t"], sim_none["y"], sim_none["r"])

    # With clamping anti-windup
    plant_clamp = MassSpringDamper()
    ctrl_clamp = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, u_min=-30.0, u_max=30.0, anti_windup="clamp")
    sim_clamp = simulate(plant_clamp, ctrl_clamp, setpoint=1.0, T=5.0)
    m_clamp = calc_metrics(sim_clamp["t"], sim_clamp["y"], sim_clamp["r"])

    # Windup causes overshoot ~17.6%; clamping drops it to ~0%
    assert m_none["overshoot"] > 15.0, f"Expected windup overshoot > 15%, got {m_none['overshoot']}%"
    assert m_clamp["overshoot"] < 1.0, f"Expected clamped overshoot < 1%, got {m_clamp['overshoot']}%"
    print(f"  [PASS] test_windup_clamping_effectiveness (None: {m_none['overshoot']:.1f}%, Clamp: {m_clamp['overshoot']:.1f}%)")


def test_setpoint_kick_suppression():
    """Verify D-on-measurement prevents 30,000 N spike seen in D-on-error."""
    # D on error: initial kick = Kd * (1.0 - 0.0) / dt = 30 * 1000 = 30,000 N
    plant_err = MassSpringDamper()
    ctrl_err = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, d_on="error")
    sim_err = simulate(plant_err, ctrl_err, setpoint=1.0, T=0.1)

    # D on measurement: no spike, u(0) = Kp * 1.0 = 300 N
    plant_meas = MassSpringDamper()
    ctrl_meas = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, d_on="measurement")
    sim_meas = simulate(plant_meas, ctrl_meas, setpoint=1.0, T=0.1)

    peak_err = np.max(np.abs(sim_err["u"]))
    peak_meas = np.max(np.abs(sim_meas["u"]))

    assert peak_err >= 30000.0, f"Expected D-on-error spike >= 30,000 N, got {peak_err} N"
    assert peak_meas <= 350.0, f"Expected D-on-measurement peak <= 350 N, got {peak_meas} N"
    print(f"  [PASS] test_setpoint_kick_suppression (D-on-error: {peak_err:.0f} N, D-on-meas: {peak_meas:.0f} N)")


def test_three_lags_steady_state():
    """Verify 3-lag system G(s)=1/(s+1)^3 has DC gain of 1.0."""
    lags = ThreeLags()
    dt = 0.001
    for _ in range(15000):  # 15 seconds (settling of 3 cascaded 1s lags)
        lags.step(u=1.0, dt=dt)
    assert abs(lags.output - 1.0) < 1e-3, f"3-lag output {lags.output}, expected 1.0"
    print("  [PASS] test_three_lags_steady_state")


if __name__ == "__main__":
    print("Running PID Lab Sanity Verification Suite...")
    test_holding_force_settles_at_1m()
    test_p_only_steady_state_errors()
    test_dt_sensitivity()
    test_unbounded_saturation_invariance()
    test_windup_clamping_effectiveness()
    test_setpoint_kick_suppression()
    test_three_lags_steady_state()
    print("\nALL SANITY TESTS PASSED SUCCESSFULLY! (100% agreement with theory & reference benchmarks)")
