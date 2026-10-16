import matplotlib.pyplot as plt
import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import flybyPointF32
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass, macros, unitTestSupport

R0 = np.array([-5e7, 7.5e6, 5e5])  # [m] initial position of the straight-line flyby
V0 = np.array([2e4, 0.0, 0.0])  # [m/s] its constant velocity; closest approach ~7.5e6 m at ~42 min
SIM_DT = 10  # [s] control period
NUM_PERIODS = 60  # diagnostic tests: 10 min before closest approach
TRIGGER_INDICES = [1, 3, 8, 10, 11, 12, 13, 14, 15, 23, 24, 25, 26, 27, 28]  # isolated periods and consecutive runs


def truth_position(i):
    """[m] position on the straight-line flyby at control period i."""
    return R0 + V0 * (i * SIM_DT)


def run_flyby(samples, filter_periods=1, orbit_normal_sign=1, max_rate=1.0, max_acceleration=1.0, num_sigmas=3.0,
              pos_variance=1e18, vel_variance=1e16, ground_state=(R0, V0)):
    """Run the module once per (r, v) sample from the given ground state; return the attitude and diagnostic recorders.
    The ground variances apply to every axis; the defaults never trip the state knowledge check."""
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process = unit_test_sim.CreateNewProcess("unit_process")
    test_process.addTask(unit_test_sim.CreateNewTask("unit_task", macros.sec2nano(SIM_DT)))

    flyby_guidance = flybyPointF32.FlybyPoint()
    flyby_guidance.modelTag = "flybyPoint"
    flyby_guidance.controlPeriod = SIM_DT
    flyby_guidance.filterReadPeriods = filter_periods
    flyby_guidance.toleranceForCollinearity = 1E-5
    flyby_guidance.signOfOrbitNormalFrameVector = orbit_normal_sign
    flyby_guidance.maximumRateThreshold = max_rate
    flyby_guidance.maximumAccelerationThreshold = max_acceleration
    flyby_guidance.numberOfSigmas = num_sigmas
    flyby_guidance.groundPosition_BN_N = np.asarray(ground_state[0], dtype=float).tolist()
    flyby_guidance.groundVelocity_BN_N = np.asarray(ground_state[1], dtype=float).tolist()
    flyby_guidance.groundPositionCovarianceDiag_N = [pos_variance] * 3
    flyby_guidance.groundVelocityCovarianceDiag_N = [vel_variance] * 3
    unit_test_sim.AddModelToTask("unit_task", flyby_guidance)

    input_data = messaging.NavTransMsgF32Payload()
    filter_msg = messaging.NavTransMsgF32()
    flyby_guidance.filterInMsg.subscribeTo(filter_msg)

    attitude_reference_log = flyby_guidance.attRefOutMsg.recorder()
    flyby_diagnostic_log = flyby_guidance.flybyDiagnosticOutMsg.recorder()
    unit_test_sim.AddModelToTask("unit_task", attitude_reference_log)
    unit_test_sim.AddModelToTask("unit_task", flyby_diagnostic_log)

    unit_test_sim.InitializeSimulation()
    for i, (r_BN_N, v_BN_N) in enumerate(samples):
        input_data.timeTag = macros.sec2nano(i * SIM_DT)
        input_data.r_BN_N = np.array(r_BN_N)
        input_data.v_BN_N = np.array(v_BN_N)
        filter_msg.write(input_data, unit_test_sim.TotalSim.getCurrentNanos())
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * SIM_DT) - 1)
        unit_test_sim.ExecuteSimulation()
    return attitude_reference_log, flyby_diagnostic_log


def trigger_periods(flags):
    """Control periods on which a diagnostic flag is set."""
    return [i for i, flag in enumerate(flags) if flag]


def expect_triggers(diagnostic_log, collinearity=(), max_rate=(), max_acceleration=(), state_knowledge=()):
    """Each validity trigger is set on exactly the given control periods."""
    np.testing.assert_equal(trigger_periods(diagnostic_log.collinearityTrigger), list(collinearity))
    np.testing.assert_equal(trigger_periods(diagnostic_log.maxRateTrigger), list(max_rate))
    np.testing.assert_equal(trigger_periods(diagnostic_log.maxAccelerationTrigger), list(max_acceleration))
    np.testing.assert_equal(trigger_periods(diagnostic_log.stateKnowledgeExceedTrigger), list(state_knowledge))


def speed_over_closest_approach(r_BN_N, v_BN_N):
    """[rad/s] |v| / d_CA, with d_CA = |r x v| / |v| the closest-approach distance of the straight line."""
    speed = np.linalg.norm(v_BN_N)
    return speed / (np.linalg.norm(np.cross(r_BN_N, v_BN_N)) / speed)


def peak_rate(r_BN_N, v_BN_N):
    """[deg/s] predicted peak frame rate |v| / d_CA."""
    return np.degrees(speed_over_closest_approach(r_BN_N, v_BN_N))


def peak_acceleration(r_BN_N, v_BN_N):
    """[deg/s^2] predicted peak frame acceleration 3 sqrt(3) / 8 (|v| / d_CA)^2."""
    return np.degrees(3.0 * np.sqrt(3.0) / 8.0 * speed_over_closest_approach(r_BN_N, v_BN_N) ** 2)


def expect_straight_line_reference(attitude_reference_log, orbit_normal_sign=1):
    """The reference follows the analytic straight-line profile of (R0, V0) on every control period: frame axes from
    the true position (rotated 180 deg about r_hat for sign -1), and rate and acceleration about the orbit normal.
    """
    uh = np.cross(R0, V0) / np.linalg.norm(np.cross(R0, V0))
    ur0 = R0 / np.linalg.norm(R0)
    ut0 = np.cross(uh, ur0)
    f0 = np.linalg.norm(V0) / np.linalg.norm(R0)
    gamma0 = np.arctan(np.dot(V0, ur0) / np.dot(V0, ut0))
    for i, sigma_RN in enumerate(attitude_reference_log.sigma_RN):
        ur = truth_position(i) / np.linalg.norm(truth_position(i))
        ut = np.cross(uh, ur)
        dcm_NR = rbk.MRP2C(sigma_RN).transpose()
        np.testing.assert_allclose(dcm_NR @ [1, 0, 0], ur, rtol=0, atol=1E-5)
        np.testing.assert_allclose(dcm_NR @ [0, 1, 0], orbit_normal_sign * ut, rtol=0, atol=1E-5)
        np.testing.assert_allclose(dcm_NR @ [0, 0, 1], orbit_normal_sign * uh, rtol=0, atol=1E-5)

        t = i * SIM_DT
        den = (f0 * t) ** 2 + 2 * f0 * np.sin(gamma0) * t + 1
        theta_dot = f0 * np.cos(gamma0) / den
        theta_ddot = -2 * f0 * f0 * np.cos(gamma0) * (f0 * t + np.sin(gamma0)) / den ** 2
        np.testing.assert_allclose(attitude_reference_log.omega_RN_N[i], uh * theta_dot, rtol=1E-4, atol=1E-10)
        np.testing.assert_allclose(attitude_reference_log.domega_RN_N[i], uh * theta_ddot, rtol=1E-4, atol=1E-10)


@pytest.mark.parametrize("filter_periods", [1, 6])
@pytest.mark.parametrize("orbit_normal_sign", [1, -1])
@pytest.mark.parametrize("reject", [False, True])
def test_flybyPoint(show_plots, filter_periods, orbit_normal_sign, reject):
    r"""
    The module follows an ideal straight-line flyby. With every re-read rejected (reject=True), the attitude comes from
    the configured ground state alone and must be the same.
    """
    num_periods = round(9 * 600 / SIM_DT)
    samples = [(truth_position(i), V0) for i in range(num_periods)]
    attitude_log, diagnostic_log = run_flyby(samples, filter_periods=filter_periods,
                                             orbit_normal_sign=orbit_normal_sign, max_rate=0.01 if reject else 1.0)

    re_read_periods = list(range(filter_periods - 1, num_periods, filter_periods))  # windows end every N periods
    expect_triggers(diagnostic_log, max_rate=re_read_periods if reject else [])
    expect_straight_line_reference(attitude_log, orbit_normal_sign)

    if show_plots:
        time_data = attitude_log.times() * macros.NANO2MIN
        plot_position(time_data, np.array([r for r, _ in samples]))
        plot_velocity(time_data, np.array([v for _, v in samples]))
        plot_ref_attitude(time_data, attitude_log.sigma_RN)
        plot_ref_rates(time_data, attitude_log.omega_RN_N)
        plot_ref_accelerations(time_data, attitude_log.domega_RN_N)
        plt.show()
    plt.close("all")


def test_flybyPoint_config_round_trip():
    r"""Every configuration property reads back what was set through SWIG."""
    flyby_guidance = flybyPointF32.FlybyPoint()
    floats = {"controlPeriod": 10.0, "toleranceForCollinearity": 1E-5, "maximumRateThreshold": 0.01,
              "maximumAccelerationThreshold": 1E-7, "numberOfSigmas": 3.0}
    for name, value in floats.items():
        setattr(flyby_guidance, name, value)
        np.testing.assert_allclose(getattr(flyby_guidance, name), value, rtol=1E-6, atol=0, err_msg=name)
    flyby_guidance.filterReadPeriods = 6
    flyby_guidance.signOfOrbitNormalFrameVector = -1
    np.testing.assert_equal(flyby_guidance.filterReadPeriods, 6)
    np.testing.assert_equal(flyby_guidance.signOfOrbitNormalFrameVector, -1)
    vectors = {"groundPosition_BN_N": [-5E7, 7.5E6, 5E5], "groundVelocity_BN_N": [2E4, 0.0, 0.0],
               "groundPositionCovarianceDiag_N": [1E8, 2E8, 3E8], "groundVelocityCovarianceDiag_N": [1.0, 2.0, 3.0]}
    for name, value in vectors.items():
        setattr(flyby_guidance, name, value)
        np.testing.assert_allclose(np.array(getattr(flyby_guidance, name)).flatten(), value, rtol=0, atol=0,
                                   err_msg=name)


def test_flybyPoint_diagnostic_collinearity():
    r"""
    Samples with velocity along r raise the collinearity, rate and acceleration flags on exactly those periods and are
    ignored.
    """
    speed = np.linalg.norm(V0)
    samples = []
    for i in range(NUM_PERIODS):
        r_BN_N = truth_position(i)
        v_BN_N = r_BN_N / np.linalg.norm(r_BN_N) * speed if i in TRIGGER_INDICES else V0
        samples.append((r_BN_N, v_BN_N))
    attitude_log, diagnostic_log = run_flyby(samples)

    expect_triggers(diagnostic_log, collinearity=TRIGGER_INDICES, max_rate=TRIGGER_INDICES,
                    max_acceleration=TRIGGER_INDICES)
    expect_straight_line_reference(attitude_log)


# limit under test
PEAK_LIMITS = {
    "rate": ("max_rate", "max_acceleration", peak_rate, "maxRateTrigger"),
    "acceleration": ("max_acceleration", "max_rate", peak_acceleration, "maxAccelerationTrigger"),
}


@pytest.mark.parametrize("limit", ["rate", "acceleration"])
def test_flybyPoint_peak_limit(limit):
    r"""
    A rate (or acceleration) limit 1% above the true peak raises no flag and 1% below flags every update; samples whose
    speed is 1000x too high are flagged and ignored.
    """
    limit_key, other_key, peak, flag = PEAK_LIMITS[limit]
    straight_line = [(truth_position(i), V0) for i in range(NUM_PERIODS)]
    for factor, expected in [(1.01, []), (0.99, list(range(NUM_PERIODS)))]:
        _, diagnostic_log = run_flyby(straight_line, **{limit_key: factor * peak(R0, V0), other_key: 1E6})
        np.testing.assert_equal(trigger_periods(getattr(diagnostic_log, flag)), expected)

    fast = [(truth_position(i), V0 * 1000.0 if i in TRIGGER_INDICES else V0) for i in range(NUM_PERIODS)]
    attitude_log, diagnostic_log = run_flyby(fast, **{limit_key: 1.0, other_key: 1E6})
    expect_triggers(diagnostic_log, **{limit_key: TRIGGER_INDICES})
    expect_straight_line_reference(attitude_log)


def test_flybyPoint_diagnostic_state_knowledge():
    r"""
    Samples farther from the ground trajectory than the propagated sigma, in position or velocity, are flagged and
    ignored: a 1 km position sigma accepts 0.99 km and rejects 1.01 km, and a 1 m/s velocity sigma rejects 2 m/s.
    """
    sigma = 1E3
    offset = np.array([0.0, 0.0, 1.0])
    for factor, expected in [(0.99, []), (1.01, list(range(1, NUM_PERIODS)))]:
        samples = [(truth_position(i) + (factor * sigma * offset if i > 0 else 0), V0) for i in range(NUM_PERIODS)]
        _, diagnostic_log = run_flyby(samples, num_sigmas=1.0, pos_variance=sigma ** 2, vel_variance=1E-6)
        expect_triggers(diagnostic_log, state_knowledge=expected)

    samples = [(truth_position(i) + (2 * sigma * offset if i in TRIGGER_INDICES else 0), V0) for i in
               range(NUM_PERIODS)]
    attitude_log, diagnostic_log = run_flyby(samples, num_sigmas=1.0, pos_variance=sigma ** 2, vel_variance=1E-6)
    expect_triggers(diagnostic_log, state_knowledge=TRIGGER_INDICES)
    expect_straight_line_reference(attitude_log)

    velocity_offset = np.array([0.0, 2.0, 0.0])
    samples = [(truth_position(i), V0 + (velocity_offset if i in TRIGGER_INDICES else 0)) for i in range(NUM_PERIODS)]
    attitude_log, diagnostic_log = run_flyby(samples, num_sigmas=1.0, pos_variance=1E18, vel_variance=1.0)
    expect_triggers(diagnostic_log, state_knowledge=TRIGGER_INDICES)
    expect_straight_line_reference(attitude_log)


def test_flybyPoint_unusable_sample(filter_periods=3, unusable_index=4):
    r"""
    An unusable sample (zero position) is flagged on its period, counted at the end of its window, and doesn't affect
    the attitude.
    """
    num_steps = 4 * filter_periods + 1
    samples = [(np.zeros(3) if i == unusable_index else truth_position(i), V0) for i in range(num_steps)]
    attitude_reference_log, flyby_diagnostic_log = run_flyby(samples, filter_periods=filter_periods)

    window_end_index = (unusable_index // filter_periods + 1) * filter_periods - 1  # windows end every N periods
    expected_rejected = [i == unusable_index for i in range(num_steps)]
    expected_count = [1 if i == window_end_index else 0 for i in range(num_steps)]
    np.testing.assert_equal(np.array(flyby_diagnostic_log.inputSampleRejected, dtype=bool), expected_rejected)
    np.testing.assert_equal(np.array(flyby_diagnostic_log.rejectedSamplesInWindow), expected_count)
    expect_straight_line_reference(attitude_reference_log)


def plot_position(time_data, position_data):
    """Plot the inertial position."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(time_data, position_data[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$r_{BN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Position [m]')


def plot_velocity(time_data, velocity_data):
    """Plot the inertial velocity."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(time_data, velocity_data[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$v_{BN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Velocity [m/s]')


def plot_ref_attitude(time_data, sigma_RN):
    """Plot the reference attitude."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(time_data, sigma_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference attitude')


def plot_ref_rates(time_data, omega_RN):
    """Plot the reference angular rates."""
    plt.figure(4)
    for idx in range(3):
        plt.plot(time_data, omega_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference rates')


def plot_ref_accelerations(time_data, omegaDot_RN):
    """Plot the reference angular accelerations."""
    plt.figure(5)
    for idx in range(3):
        plt.plot(time_data, omegaDot_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\dot{\omega}_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference accelerations')


if __name__ == "__main__":
    test_flybyPoint(True, 6, 1, False)
