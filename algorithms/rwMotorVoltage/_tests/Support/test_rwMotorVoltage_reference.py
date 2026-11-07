"""rwMotorVoltage reference values (port of unitTestSupport.nb).

Derives the expected wheel voltages that ``test_rwMotorVoltage.py`` hardcodes in ``voltage_true``:

- Test 1: linear torque-to-voltage map with a ``VMin`` dead-band offset.
- Test 2: the same map, with the result clamped to ``[-VMax, VMax]``.
- torqueLoop: closed-loop torque correction from the measured wheel-speed change,
  before and after a module reset.

Run ``python test_rwMotorVoltage_reference.py`` to print the walkthrough. Run with pytest to check the
values against the notebook outputs.
"""

import numpy as np

# --- Setup ------------------------------------------------------------------------------------------
JS = 0.1  # wheel spin inertia [kg m^2]
U_MAX = 0.2  # maximum wheel torque [N m]
V_MIN = 1.0  # minimum (dead-band) voltage [V]
V_MAX = 11.0  # maximum voltage [V]

# torqueLoop setup
K = 1.5  # torque-loop feedback gain
OMEGA = np.array([1.0, 2.0, 1.5, -3.0])  # wheel speeds at the prior step [rad/s]
OMEGA1 = np.array([1.1, 2.1, 1.1, -4.1])  # wheel speeds at the current step [rad/s]
DT = 0.5  # time step [s]

US_TEST1 = np.array([0.05, 0.0, -0.15, -0.2])
US_TEST2 = np.array([0.5, 0.0, -0.15, -0.5])
US_TORQUE_LOOP = US_TEST1


# --- Formulas ---------------------------------------------------------------------------------------
def torque_to_voltage(us):
    """V = (VMax - VMin)/uMax * us + VMin * sign(V). Not clamped."""
    v = (V_MAX - V_MIN) / U_MAX * us
    return v + V_MIN * np.sign(v)


def clamp_voltage(v):
    """Clamp each voltage to [-VMax, VMax] (notebook Test 2 ``Do`` loop)."""
    return np.clip(v, -V_MAX, V_MAX)


def actual_torque(omega, omega1, dt):
    """tau_actual = Js * (Omega1 - Omega) / dt."""
    return JS * (omega1 - omega) / dt


def ideal_wheel_speed(omega, us, dt):
    """Omega_ideal = Omega + us / Js * dt. The notebook computes this for reference only."""
    return omega + us / JS * dt


def closed_loop_torque(us, tau_actual):
    """usCL = us - K * (tau_actual - us)."""
    return us - K * (tau_actual - us)


def torque_loop(omega, omega1):
    """Return (tau_actual, Omega_ideal, usCL, V) for one torqueLoop step."""
    tau = actual_torque(omega, omega1, DT)
    omega_ideal = ideal_wheel_speed(omega, US_TORQUE_LOOP, DT)
    us_cl = closed_loop_torque(US_TORQUE_LOOP, tau)
    return tau, omega_ideal, us_cl, torque_to_voltage(us_cl)


# --- Notebook outputs -------------------------------------------------------------------------------
V_TEST1 = [3.5, 0.0, -8.5, -11.0]
V_TEST2 = [11.0, 0.0, -8.5, -11.0]

# Before reset: Omega and Omega1 differ, so the loop sees a measured torque.
TAU_BEFORE = [0.020000000000000018, 0.020000000000000018, -0.07999999999999999, -0.21999999999999995]
OMEGA_IDEAL_BEFORE = [1.25, 2.0, 0.75, -4.0]
US_CL_BEFORE = [0.09499999999999997, -0.030000000000000027, -0.255, -0.1700000000000001]
V_BEFORE = [5.749999999999998, -2.5000000000000013, -13.75, -9.500000000000005]

# After reset: Omega = Omega1, so the measured torque is zero.
TAU_AFTER = [0.0, 0.0, 0.0, 0.0]
OMEGA_IDEAL_AFTER = [1.35, 2.1, 0.3500000000000001, -5.1]
US_CL_AFTER = [0.125, 0.0, -0.375, -0.5]
V_AFTER = [7.25, 0.0, -19.75, -26.0]

# The notebook does not clamp the torqueLoop voltages. The module does, so
# test_rwMotorVoltage.py (use_torque_loop rows 4 and 7 of voltage_true) expects the clamped values.
V_BEFORE_CLAMPED = [5.75, -2.5, -11.0, -9.5]
V_AFTER_CLAMPED = [7.25, 0.0, -11.0, -11.0]


# --- Checks -----------------------------------------------------------------------------------------
def _check(actual, desired):
    np.testing.assert_allclose(actual, desired, rtol=1e-12, atol=1e-15)


def test_test1_voltage():
    _check(torque_to_voltage(US_TEST1), V_TEST1)


def test_test2_clamped_voltage():
    _check(clamp_voltage(torque_to_voltage(US_TEST2)), V_TEST2)


def test_torque_loop_before_reset():
    tau, omega_ideal, us_cl, v = torque_loop(OMEGA, OMEGA1)
    _check(tau, TAU_BEFORE)
    _check(omega_ideal, OMEGA_IDEAL_BEFORE)
    _check(us_cl, US_CL_BEFORE)
    _check(v, V_BEFORE)
    _check(clamp_voltage(v), V_BEFORE_CLAMPED)


def test_torque_loop_after_reset():
    tau, omega_ideal, us_cl, v = torque_loop(OMEGA1, OMEGA1)
    _check(tau, TAU_AFTER)
    _check(omega_ideal, OMEGA_IDEAL_AFTER)
    _check(us_cl, US_CL_AFTER)
    _check(v, V_AFTER)
    _check(clamp_voltage(v), V_AFTER_CLAMPED)


if __name__ == "__main__":
    np.set_printoptions(precision=17)
    print("Test 1: V =", torque_to_voltage(US_TEST1))
    print("Test 2: V =", clamp_voltage(torque_to_voltage(US_TEST2)))
    for label, (omega, omega1) in (("before reset", (OMEGA, OMEGA1)), ("after reset", (OMEGA1, OMEGA1))):
        tau, omega_ideal, us_cl, v = torque_loop(omega, omega1)
        print(f"torqueLoop ({label}):")
        print("  tau_actual  =", tau)
        print("  Omega_ideal =", omega_ideal)
        print("  usCL        =", us_cl)
        print("  V           =", v)
        print("  V (clamped) =", clamp_voltage(v))
