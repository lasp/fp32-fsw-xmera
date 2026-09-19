# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Python integration tests for the flybyFilter fp32 module. Exercises the full xmera adapter path
# (SWIG, SI<->km unit conversion, message I/O) using the same optical-navigation heading measurement
# as fswAlgorithms/opticalNavigation/flybyODuKF: a unit vector rhat_BN_N with measurement model r/|r|.
#
# Scenarios: pure propagation against an rk4 truth, heading tracking through a mid-arc velocity kick,
# navTransOutMsg agreement with the filter state, validity/freshness gating of the heading port,
# delayed-measurement anchoring, and unit-conversion invariance of the SI output.
#
# Not covered here: the adapter's error paths (unlinked message, non-positive unitConversion,
# lifecycle calls before reset, invalid configuration). No fp32 SWIG module declares exception
# translation, so a C++ throw crossing into Python calls std::terminate and would abort the whole
# pytest run rather than raising. Those paths are covered by the C++ suite where they are reachable.

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

    opnav_msg = messaging.OpNavUnitVecMsgF32()
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
    filter_plots.covar_trace(covar_log, num_states, 'Prop', show_plots)

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

    opnav_payload = messaging.OpNavUnitVecMsgF32Payload()
    opnav_msg = messaging.OpNavUnitVecMsgF32()
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
    error_log = np.copy(state_log)
    error_log[:, 1:] -= truth[:len(error_log), 1:]
    filter_plots.states(error_log, 'Update error', show_plots)
    filter_plots.state_covar(state_log, covar_log, 'Update', show_plots)
    filter_plots.covar_trace(covar_log, num_states, 'Update', show_plots)
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


def _run_heading_scenario(unit_conversion, headings, n_steps=60, dt=1.0):
    """Run the adapter for n_steps, writing the heading payloads supplied by `headings(i)` (which
    returns None for a step with no fresh reading). Returns (module, filter_log, res_log, nav_log)."""
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("test_process")
    proc.addTask(sim.CreateNewTask("unit_task", macros.sec2nano(dt)))

    module = flybyFilterF32.FlybyFilter()
    sim.AddModelToTask("unit_task", module)

    r0, v0 = truth_rv()
    setup_filter_data(module, np.concatenate([r0, v0]))
    module.unitConversion = unit_conversion

    filter_log = module.filterOutMsg.recorder()
    res_log = module.filterResOutMsg.recorder()
    nav_log = module.navTransOutMsg.recorder()
    for log in (filter_log, res_log, nav_log):
        sim.AddModelToTask("unit_task", log)

    opnav_msg = messaging.OpNavUnitVecMsgF32()
    module.opNavHeadingMsg.subscribeTo(opnav_msg)

    sim.InitializeSimulation()
    for i in range(n_steps):
        payload = headings(i)
        if payload is not None:
            opnav_msg.write(payload, sim.TotalSim.getCurrentNanos())
        sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        sim.ExecuteSimulation()

    return module, filter_log, res_log, nav_log


def _heading_payload(time_tag, rhat, valid=True):
    payload = messaging.OpNavUnitVecMsgF32Payload()
    payload.timeTag = time_tag
    payload.rhat_BN_N = list(rhat)
    payload.valid = valid
    return payload


def test_nav_trans_output_matches_the_filter_state(show_plots):
    """navTransOutMsg must carry the same SI position and velocity as filterOutMsg's first six
    states, with a matching time tag. Nothing else in the suite reads this port."""
    del show_plots
    r0, v0 = truth_rv()
    rhat0 = r0 / np.linalg.norm(r0)

    _, filter_log, _, nav_log = _run_heading_scenario(
        1E-3, lambda i: _heading_payload((i + 1) * 1.0, rhat0) if i % 10 == 0 else None)

    num_states = filter_log.numberOfStates[0]
    state = np.array(filter_log.state)[:, :num_states]
    r_out = np.array(nav_log.r_BN_N)
    v_out = np.array(nav_log.v_BN_N)

    np.testing.assert_allclose(r_out, state[:, 0:3], rtol=1E-12, atol=0.0,
                               err_msg="navTransOutMsg position must match the filter state")
    np.testing.assert_allclose(v_out, state[:, 3:6], rtol=1E-12, atol=0.0,
                               err_msg="navTransOutMsg velocity must match the filter state")
    np.testing.assert_allclose(np.array(nav_log.timeTag), np.array(filter_log.timeTag), rtol=0.0, atol=0.0,
                               err_msg="navTransOutMsg and filterOutMsg must share a time tag")


def test_heading_is_gated_on_validity_and_freshness(show_plots):
    """The adapter only forwards a reading whose valid flag is set and whose time tag is newer than
    the last accepted one, so an invalid payload and a repeated time tag must both be ignored."""
    del show_plots
    r0, _ = truth_rv()
    rhat0 = r0 / np.linalg.norm(r0)

    # Invalid payloads only: no measurement may ever fire.
    _, _, res_invalid, _ = _run_heading_scenario(
        1E-3, lambda i: _heading_payload((i + 1) * 1.0, rhat0, valid=False))
    assert not np.array(res_invalid.valid, dtype=bool).any(), \
        "a payload with valid=False must never be applied"

    # A single time tag repeated every step: only the first delivery may fire.
    _, _, res_repeat, _ = _run_heading_scenario(
        1E-3, lambda i: _heading_payload(5.0, rhat0))
    assert np.count_nonzero(np.array(res_repeat.valid, dtype=bool)) == 1, \
        "a repeated time tag must be accepted exactly once"


def test_delayed_measurement_anchors_to_its_time_tag(show_plots):
    """A reading delivered late, but stamped newer than the filter's anchor, must be applied at its
    own time tag -- so delivering it late gives the same estimate as delivering it on time. A
    reading stamped older than the anchor must be dropped instead."""
    del show_plots
    r0, _ = truth_rv()
    rhat0 = r0 / np.linalg.norm(r0)
    meas_step, late_step = 20, 40

    def on_time(i):
        if i == 5:
            return _heading_payload(6.0, rhat0)
        if i == meas_step:
            return _heading_payload(meas_step + 1.0, rhat0)
        return None

    def delivered_late(i):
        if i == 5:
            return _heading_payload(6.0, rhat0)
        if i == late_step:
            return _heading_payload(meas_step + 1.0, rhat0)  # still stamped at its own time
        return None

    _, log_on_time, res_on_time, _ = _run_heading_scenario(1E-3, on_time)
    _, log_late, res_late, _ = _run_heading_scenario(1E-3, delivered_late)

    assert np.count_nonzero(np.array(res_late.valid, dtype=bool)) == 2, \
        "a late but past-the-anchor measurement must still be applied"
    np.testing.assert_allclose(np.array(log_late.state)[-1, :6], np.array(log_on_time.state)[-1, :6],
                               rtol=1E-9,
                               err_msg="a late measurement must land at its time tag, not its delivery time")

    # Now stamp the second reading *before* the anchor: it must be dropped entirely.
    def stale(i):
        if i == 5:
            return _heading_payload(6.0, rhat0)
        if i == late_step:
            return _heading_payload(2.0, rhat0)
        return None

    _, _, res_stale, _ = _run_heading_scenario(1E-3, stale)
    assert np.count_nonzero(np.array(res_stale.valid, dtype=bool)) == 1, \
        "a measurement stamped before the anchor must be dropped"


def test_unit_conversion_does_not_change_the_si_result(show_plots):
    """unitConversion only sets the filter's internal length scale; the SI values on the wire must
    not depend on it. Running the identical scenario in km (1e-3) and in metres (1.0) is what would
    catch a wrong exponent in the state / covariance / mu scaling."""
    del show_plots
    r0, _ = truth_rv()
    rhat0 = r0 / np.linalg.norm(r0)

    def headings(i):
        return _heading_payload((i + 1) * 1.0, rhat0) if i % 5 == 0 else None

    _, log_km, _, _ = _run_heading_scenario(1E-3, headings)
    _, log_m, _, _ = _run_heading_scenario(1.0, headings)

    num_states = log_km.numberOfStates[0]
    state_km = np.array(log_km.state)[-1, :num_states]
    state_m = np.array(log_m.state)[-1, :num_states]
    covar_km = np.array(log_km.covar)[-1, :num_states ** 2]
    covar_m = np.array(log_m.covar)[-1, :num_states ** 2]

    np.testing.assert_allclose(state_m, state_km, rtol=1E-6,
                               err_msg="the SI state must not depend on the internal unit scale")
    np.testing.assert_allclose(covar_m, covar_km, rtol=1E-6, atol=1E-9,
                               err_msg="the SI covariance must not depend on the internal unit scale")


if __name__ == "__main__":
    test_propagation(True)
    test_measurements(True)
    test_nav_trans_output_matches_the_filter_state(True)
    test_heading_is_gated_on_validity_and_freshness(True)
    test_delayed_measurement_anchors_to_its_time_tag(True)
    test_unit_conversion_does_not_change_the_si_result(True)
