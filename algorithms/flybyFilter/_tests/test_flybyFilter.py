# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Python integration test for the flybyFilter fp32 module. Exercises the full xmera adapter path
# (SWIG, SI<->km unit conversion, message I/O) using the same optical-navigation heading measurement
# as fswAlgorithms/opticalNavigation/flybyODuKF: a unit vector rhat_BN_N with measurement model r/|r|.

import flybyFilter_test_utilities as filter_plots
import numpy as np
from xmera.architecture import messaging
from xmera.fp32 import flybyFilterF32
from xmera.utilities import SimulationBaseClass, macros, orbitalMotion

MU_SI = 42828.314 * 1E9  # Mars gravitational parameter [m^3/s^2]
HEADING_STD = 1E-3       # heading (unit-vector) measurement noise std


def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))


def two_body_gravity(t, x, mu=MU_SI):
    dxdt = np.zeros(np.shape(x))
    dxdt[0:3] = x[3:]
    dxdt[3:] = -mu / np.linalg.norm(x[0:3]) ** 3.0 * x[0:3]
    return dxdt


def rk4(f, t, x0):
    x = np.zeros([len(t), len(x0) + 1])
    x[0, 0] = t[0]
    x[0, 1:] = x0
    for i in range(len(t) - 1):
        h = t[i + 1] - t[i]
        k1 = h * f(t[i], x[i, 1:])
        k2 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k1)
        k3 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k2)
        k4 = h * f(t[i] + h, x[i, 1:] + k3)
        x[i + 1, 1:] = x[i, 1:] + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        x[i + 1, 0] = t[i + 1]
    return x


def truth_rv():
    oe = orbitalMotion.ClassicElements()
    oe.a = 4000 * 1E3  # m
    oe.e = 0.2
    oe.i = 0.2
    oe.Omega = 0.001
    oe.omega = 0.01
    oe.f = 0.1
    r, v = orbitalMotion.elem2rv(MU_SI, oe)
    return np.array(r).reshape(3), np.array(v).reshape(3)


def setup_filter_data(module, initial_state_si):
    module.alpha = 0.02
    module.beta = 2.0
    module.unitConversion = 1E-3  # filter internally in km, km/s
    module.mu = MU_SI
    module.headingMeasurementNoiseStd = HEADING_STD
    module.initialState = list(initial_state_si)
    module.initialCovariance = np.diag([1000.0 * 1E6] * 3 + [0.1 * 1E6] * 3).tolist()  # m^2, (m/s)^2
    module.processNoise = np.diag([(1E-6) ** 2] * 3 + [(1E-8) ** 2] * 3).tolist()


def specific_energy(states):
    """Two-body specific orbital energy (-mu / 2a) for each row of an [N, 6] SI state array."""
    return np.array([
        -MU_SI / (2.0 * orbitalMotion.rv2elem(MU_SI, states[i, 0:3], states[i, 3:6]).a)
        for i in range(states.shape[0])
    ])


def test_propagation(show_plots):
    """No measurements: the adapter propagates a two-body orbit whose specific energy matches an
    independent rk4 truth (and is conserved), while the covariance grows. Validates the SI<->km unit
    conversion end-to-end."""
    sim = SimulationBaseClass.SimBaseClass()
    dt = 10.0
    proc = sim.CreateNewProcess("test_process")
    proc.addTask(sim.CreateNewTask("unit_task", macros.sec2nano(dt)))

    module = flybyFilterF32.FlybyFilter()
    sim.AddModelToTask("unit_task", module)

    r0, v0 = truth_rv()
    setup_filter_data(module, np.concatenate([r0, v0]))

    filter_log = module.filterOutMsg.recorder()
    sim.AddModelToTask("unit_task", filter_log)

    opnav_msg = messaging.OpNavUnitVecMsg()
    module.opNavHeadingMsg.subscribeTo(opnav_msg)  # required connection; never written (no measurements)

    sim_min = 30
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.min2nano(sim_min))
    sim.ExecuteSimulation()

    num_states = filter_log.numberOfStates[0]
    assert num_states == 6
    state_log = add_time_column(filter_log.times(), filter_log.state[:, :num_states])
    covar_log = add_time_column(filter_log.times(), filter_log.covar[:, :num_states ** 2])

    # Independent rk4 truth sampled at the recorder's own times (SI); the filter's propagated energy
    # must match the truth energy, not merely be self-consistent.
    times_s = filter_log.times() * 1.0E-9
    truth = rk4(two_body_gravity, times_s, np.concatenate([r0, v0]))
    energy_filter = specific_energy(state_log[:, 1:7])
    energy_truth = specific_energy(truth[:, 1:7])

    filter_plots.energy(times_s, energy_filter, 'Prop', show_plots)
    filter_plots.state_covar(state_log, covar_log, 'Prop', show_plots)

    assert np.all(np.isfinite(state_log[:, 1:])), "filter state must stay finite"
    np.testing.assert_allclose(energy_filter, energy_truth, rtol=1E-2, atol=1E-6,
                               err_msg="filter propagated energy must match the rk4 truth energy")
    np.testing.assert_allclose(energy_filter, energy_filter[0], rtol=1E-3, atol=1E-6,
                               err_msg="two-body energy not conserved through the adapter")
    assert np.linalg.norm(covar_log[-1, 1:]) > np.linalg.norm(covar_log[0, 1:]), \
        "covariance must grow without measurements"


def test_measurements(show_plots):
    """Feed optical-nav heading measurements along a two-body truth arc with a mid-arc velocity kick.
    The heading updates must shrink the covariance and drive the estimate to the (kicked) truth
    trajectory -- exercising state + covariance through the full adapter path (mirrors flybyODuKF)."""
    sim = SimulationBaseClass.SimBaseClass()
    dt = 1.0
    t1 = 250
    n_steps = 8 * t1
    proc = sim.CreateNewProcess("test_process")
    proc.addTask(sim.CreateNewTask("unit_task", macros.sec2nano(dt)))

    module = flybyFilterF32.FlybyFilter()
    sim.AddModelToTask("unit_task", module)

    r0, v0 = truth_rv()
    setup_filter_data(module, np.concatenate([r0, v0]))  # seed at truth; the kick creates the error

    filter_log = module.filterOutMsg.recorder()
    res_log = module.filterResOutMsg.recorder()
    sim.AddModelToTask("unit_task", filter_log)
    sim.AddModelToTask("unit_task", res_log)

    opnav_payload = messaging.OpNavUnitVecMsgPayload()
    opnav_msg = messaging.OpNavUnitVecMsg()
    module.opNavHeadingMsg.subscribeTo(opnav_msg)

    # Truth: two-body arc with a velocity kick at t1 (makes range/velocity observable from headings).
    time = np.linspace(0, n_steps * dt, n_steps + 1)
    kick = np.array([0.0, 0.0, 0.0, -0.01, 0.01, 0.02]) * 10 * 1E3  # [m, m/s]
    truth = np.zeros([n_steps + 1, 7])
    truth[0:t1] = rk4(two_body_gravity, time[0:t1], np.concatenate([r0, v0]))
    truth[t1:] = rk4(two_body_gravity, time[t1:], truth[t1 - 1, 1:] + kick)

    rng = np.random.default_rng(7)
    sim.InitializeSimulation()
    for i in range(n_steps):
        if i > 0 and i % 10 == 0:
            rhat = truth[i, 1:4] / np.linalg.norm(truth[i, 1:4])
            rhat = rhat + HEADING_STD * rng.standard_normal(3)
            rhat /= np.linalg.norm(rhat)
            opnav_payload.timeTag = i * dt
            opnav_payload.rhat_BN_N = rhat.tolist()
            opnav_payload.valid = True
            opnav_msg.write(opnav_payload, sim.TotalSim.getCurrentNanos())
        sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        sim.ExecuteSimulation()

    num_states = filter_log.numberOfStates[0]
    state_log = add_time_column(filter_log.times(), filter_log.state[:, :num_states])
    covar_log = add_time_column(filter_log.times(), filter_log.covar[:, :num_states ** 2])

    valid = np.array(res_log.valid, dtype=bool)
    pre = np.array(res_log.preFits)[:, :3]
    post = np.array(res_log.postFits)[:, :3]
    assert valid.any(), "at least one heading measurement must fire"

    res_time = add_time_column(res_log.times(), post)
    filter_plots.state_covar(state_log, covar_log, 'Update', show_plots)
    filter_plots.post_fit_residuals(res_time, HEADING_STD, 'Update', show_plots)
    filter_plots.two_orbits(truth[:, 0:4], state_log[:, 0:4], show_plots)

    def cov_trace(row):
        return float(np.trace(row[1:].reshape(num_states, num_states)))

    # Covariance shrinks as measurements are ingested (mid-arc and by the end).
    assert cov_trace(covar_log[t1]) < cov_trace(covar_log[0]), "covariance must shrink during tracking"
    assert cov_trace(covar_log[-1]) < cov_trace(covar_log[0]), "covariance must shrink by the end"
    # The estimate converges to the (kicked) truth trajectory (position + velocity).
    np.testing.assert_allclose(state_log[-1, 1:], truth[-1, 1:], rtol=1E-1,
                               err_msg="estimate must converge to the truth trajectory")
    # Informative updates also reduce the residual.
    assert np.linalg.norm(post[valid], axis=1).mean() < np.linalg.norm(pre[valid], axis=1).mean(), \
        "measurement updates should reduce the residual"


# Measurement indices (not sim steps) at which a gross heading outlier is injected.
HEADING_OUTLIER_MEAS = (20, 40, 60, 80)
HEADING_OUTLIER = np.array([0.4, -0.4, 0.4])  # [-] added to the unit vector before renormalising


def test_outlier_recovery(show_plots):
    """Feed optical-nav headings along a two-body arc seeded at truth, with occasional wildly wrong
    headings mixed in, and check the filter absorbs only a small part of each and settles back.

    A tuning instrument as much as a test: the fraction of an outlier that reaches the estimate is
    essentially the Kalman gain, which tracks the process/measurement noise ratio, and the recovery
    length is how many measurements that gain needs to work the perturbation out. Lowering the ratio
    shrinks the first and lengthens the second; the printed table is the read-out for that trade.

    The assertions are upper bounds only, so they hold whether the filter screens outliers out
    before the update or simply out-weighs them."""
    sim = SimulationBaseClass.SimBaseClass()
    dt = 1.0
    n_steps = 1000
    meas_every = 10
    proc = sim.CreateNewProcess("test_process")
    proc.addTask(sim.CreateNewTask("unit_task", macros.sec2nano(dt)))

    module = flybyFilterF32.FlybyFilter()
    sim.AddModelToTask("unit_task", module)

    r0, v0 = truth_rv()
    setup_filter_data(module, np.concatenate([r0, v0]))  # seeded at truth: the outliers are the only error

    filter_log = module.filterOutMsg.recorder()
    res_log = module.filterResOutMsg.recorder()
    sim.AddModelToTask("unit_task", filter_log)
    sim.AddModelToTask("unit_task", res_log)

    opnav_payload = messaging.OpNavUnitVecMsgPayload()
    opnav_msg = messaging.OpNavUnitVecMsg()
    module.opNavHeadingMsg.subscribeTo(opnav_msg)

    time = np.linspace(0, n_steps * dt, n_steps + 1)
    truth = rk4(two_body_gravity, time, np.concatenate([r0, v0]))

    meas_steps = list(range(meas_every, n_steps, meas_every))
    heading_meas = np.zeros([len(meas_steps), 4])
    heading_truth = np.zeros([len(meas_steps), 4])

    rng = np.random.default_rng(7)
    sim.InitializeSimulation()
    for i in range(n_steps):
        if i in meas_steps:
            n = meas_steps.index(i)
            clean = truth[i, 1:4] / np.linalg.norm(truth[i, 1:4])
            rhat = clean + HEADING_STD * rng.standard_normal(3)
            if n in HEADING_OUTLIER_MEAS:
                rhat = rhat + HEADING_OUTLIER
            rhat /= np.linalg.norm(rhat)
            heading_meas[n] = [macros.sec2nano(time[i]), *rhat]
            heading_truth[n] = [macros.sec2nano(time[i]), *clean]
            opnav_payload.timeTag = i * dt
            opnav_payload.rhat_BN_N = rhat.tolist()
            opnav_payload.valid = True
            opnav_msg.write(opnav_payload, sim.TotalSim.getCurrentNanos())
        sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        sim.ExecuteSimulation()

    num_states = 6
    state_log = add_time_column(filter_log.times(), filter_log.state[:, :num_states])
    covar_log = add_time_column(filter_log.times(), filter_log.covar[:, :num_states ** 2])
    res_post = add_time_column(res_log.times(), np.array(res_log.postFits)[:, :3])

    # Sample the error on the measurement grid, so "recovery" is counted in measurements rather than
    # in sim steps.
    pos_err = np.array([np.linalg.norm(state_log[i, 1:4] - truth[i, 1:4]) for i in meas_steps])
    err_col = np.column_stack([heading_meas[:, 0], pos_err])

    warmup = 5
    quiet = [k for k in range(warmup, len(pos_err))
             if all(abs(k - o) > 3 for o in HEADING_OUTLIER_MEAS)]
    ambient = float(np.median(pos_err[quiet]))

    peak_bound = 10.0
    recovery_bound = 15  # measurements
    report = []
    for n in HEADING_OUTLIER_MEAS:
        peak = float(np.max(pos_err[n:n + 2]))
        recovery = next((d for d in range(1, len(pos_err) - n) if pos_err[n + d] < 2 * ambient), None)
        report.append((n, peak, peak / ambient, recovery))

    print("FlybyFilter outlier recovery")
    print("  ambient position error: %.3e m   [headingMeasurementNoiseStd %.3e]" % (ambient, HEADING_STD))
    print("  meas  peak err     x ambient   recovery")
    for n, peak, xamb, recovery in report:
        print("  %4d  %.3e   %6.1fx    %s meas" % (n, peak, xamb, recovery))

    filter_plots.outlier_rejection(heading_meas, heading_truth,
                                   np.ones(len(meas_steps), dtype=bool), 'Heading Outliers', show_plots)
    filter_plots.error_recovery(err_col, HEADING_OUTLIER_MEAS, ambient, 'Position', show_plots)
    filter_plots.state_covar(state_log, covar_log, 'Outlier Recovery', show_plots)
    filter_plots.post_fit_residuals(res_post, HEADING_STD, 'Outlier Recovery', show_plots)

    for n, peak, xamb, recovery in report:
        assert peak <= peak_bound * ambient, \
            'the outlier at measurement %d moved the estimate to %g, beyond %gx the ambient error %g' \
            % (n, peak, peak_bound, ambient)
        assert recovery is not None and recovery <= recovery_bound, \
            'the estimate did not settle back within %d measurements of the outlier at measurement %d' \
            % (recovery_bound, n)

    assert np.all(np.isfinite(state_log[:, 1:])), "filter state must stay finite"


if __name__ == "__main__":
    test_propagation(False)
    test_measurements(False)
    test_outlier_recovery(False)
