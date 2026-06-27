# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Unit tests for the inertial attitude square-root UKF (fp32). Structured after the
# Basilisk inertialUKF test (attDetermination/InertialUKF/_UnitTest/test_inertialKF.py),
# but the filter dynamics are MRP attitude kinematics (state = [sigma_BN, omega_BN_B])
# and the measurements/messages are the fp32 SRuKF ones: a star-tracker attitude input
# (STAttMsgPayload) and a gyro buffer input (AccDataMsgPayload), with the estimate and
# covariance read back from filterOutMsg (FilterMsgPayload).

import inertialSRuKF_test_utilities as filter_plots
import numpy as np
from xmera.architecture import messaging
from xmera.fp32 import inertialSRuKFF32
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass, macros
from xmera.utilities import unitTestSupport

NUM_STATES = 6


def rk4(f, t, x0, mrpShadow=False):
    x = np.zeros([len(t), len(x0) + 1])
    x[0, 0] = t[0]
    x[0, 1:] = x0
    for i in range(len(t) - 1):
        h = t[i + 1] - t[i]
        x[i, 0] = t[i]
        k1 = h * f(t[i], x[i, 1:])
        k2 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k1)
        k3 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k2)
        k4 = h * f(t[i] + h, x[i, 1:] + k3)
        x[i + 1, 1:] = x[i, 1:] + (k1 + 2. * k2 + 2. * k3 + k4) / 6.
        if mrpShadow and np.linalg.norm(x[i + 1, 1:4]) ** 2 > 1:
            x[i + 1, 1:4] = -(x[i + 1, 1:4]) / np.linalg.norm(x[i + 1, 1:4]) ** 2
        x[i + 1, 0] = t[i + 1]
    return x


def mrp_integration(t, x):
    # Attitude kinematics: sigma_dot = 1/4 B(sigma) omega; rate constant.
    dxdt = np.zeros(np.shape(x))
    dxdt[:3] = 0.25 * np.matmul(rbk.BmatMRP(x[0:3]), x[3:6])
    dxdt[3:6] = np.zeros(3)
    return dxdt


def setupFilterData(filterObject):
    filterObject.alpha = 0.02
    filterObject.beta = 2.0

    filterObject.initialState = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    covarInit = np.zeros((NUM_STATES, NUM_STATES))
    np.fill_diagonal(covarInit, [0.04, 0.04, 0.04, 0.004, 0.004, 0.004])
    filterObject.initialCovariance = covarInit.tolist()

    qNoiseIn = np.identity(NUM_STATES)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 0.0017 * 0.0017
    qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 0.00017 * 0.00017
    filterObject.processNoise = qNoiseIn.tolist()

    filterObject.stMeasurementNoiseStd = 0.00017
    filterObject.gyroMeasurementNoiseStd = 0.0017


def test_stateUpdateInertialAttitude(show_plots):
    [testResults, testMessage] = stateUpdateInertialAttitude(show_plots)
    assert testResults < 1, testMessage


def stateUpdateInertialAttitude(show_plots):
    """Feed a constant star-tracker attitude and check the estimate converges to it
    while the covariance shrinks. A second phase re-points to a different attitude to
    confirm the filter re-acquires it.

    Note: this kinematic SRuKF forms the star-tracker innovation from the relative MRP
    (subMrp) with a linear state update and uses linear sigma-point means, so it is valid
    for moderate attitude errors. Unlike the Basilisk inertialUKF it does not switch MRP
    sigma points across the |sigma| = 1 boundary, so over-unity MRP measurements are out
    of scope here; both phases stay comfortably inside the unit sphere."""
    testFailCount = 0
    testMessages = []

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    dt = 0.5
    testProcessRate = macros.sec2nano(dt)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = inertialSRuKFF32.InertialSRuKF()
    unitTestSim.AddModelToTask(unitTaskName, module)
    setupFilterData(module)

    filterLog = module.filterOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, filterLog)

    stMessage = messaging.STAttMsgPayload()
    stMessage.MRP_BdyInrtl = [0.3, 0.4, 0.5]
    stInMsg = messaging.STAttMsg()
    module.stAttInMsg.subscribeTo(stInMsg)

    unitTestSim.InitializeSimulation()

    num_steps = 400
    for i in range(num_steps):
        if i > 21:
            stMessage.timeTag = i * dt  # [s] adapter compares timeTag in seconds
            stInMsg.write(stMessage, macros.sec2nano(i * dt))
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.state[:, :NUM_STATES])
    covarLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.covar[:, :NUM_STATES ** 2])

    accuracy = 1.0E-4
    for i in range(3):
        if covarLog[-1, i * NUM_STATES + i + 1] > covarLog[0, i * NUM_STATES + i + 1]:
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if abs(stateLog[-1, i + 1] - stMessage.MRP_BdyInrtl[i]) > accuracy:
            testFailCount += 1
            testMessages.append("State update failure")

    # Second phase: re-point to a different attitude and confirm the filter re-acquires it.
    stMessage.MRP_BdyInrtl = [-0.2, 0.3, -0.1]
    for i in range(num_steps):
        stMessage.timeTag = (i + num_steps) * dt
        stInMsg.write(stMessage, macros.sec2nano((i + num_steps) * dt))
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + num_steps + 1) * dt))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.state[:, :NUM_STATES])
    covarLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.covar[:, :NUM_STATES ** 2])
    for i in range(3):
        if covarLog[-1, i * NUM_STATES + i + 1] > covarLog[0, i * NUM_STATES + i + 1]:
            testFailCount += 1
            testMessages.append("Covariance update large failure")
        if abs(stateLog[-1, i + 1] - stMessage.MRP_BdyInrtl[i]) > accuracy:
            testFailCount += 1
            testMessages.append("State re-point failure")

    if show_plots:
        filter_plots.state_covar(stateLog, covarLog, 'Update').show()

    if testFailCount == 0:
        print("PASSED: InertialSRuKF state update")
    return [testFailCount, ''.join(testMessages)]


def test_stateUpdateRate(show_plots):
    [testResults, testMessage] = stateUpdateRate(show_plots)
    assert testResults < 1, testMessage


def stateUpdateRate(show_plots):
    """Feed gyro rate measurements and the matching rotating star-tracker attitude, and
    check the rate estimate converges to the gyro rate while the attitude tracks truth."""
    testFailCount = 0
    testMessages = []

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    dt = 0.5
    testProcessRate = macros.sec2nano(dt)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = inertialSRuKFF32.InertialSRuKF()
    unitTestSim.AddModelToTask(unitTaskName, module)
    setupFilterData(module)

    filterLog = module.filterOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, filterLog)
    stResLog = module.filterStResOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, stResLog)
    gyroResLog = module.filterGyroResOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, gyroResLog)

    # Truth: constant body rate, attitude integrated from the MRP kinematics.
    truthRate = np.array([0.02, -0.01, 0.015])
    num_steps = 600
    time = np.linspace(0, num_steps * dt, num_steps + 1)
    truth = rk4(mrp_integration, time, np.array([0.0, 0.0, 0.0, *truthRate]), mrpShadow=True)

    stMessage = messaging.STAttMsgPayload()
    stInMsg = messaging.STAttMsg()
    module.stAttInMsg.subscribeTo(stInMsg)

    gyroBuffer = messaging.AccDataMsgPayload()
    gyroInMsg = messaging.AccDataMsg()
    module.gyrBuffInMsg.subscribeTo(gyroInMsg)

    np.random.seed(0)
    stSigma = module.stMeasurementNoiseStd
    gyroSigma = module.gyroMeasurementNoiseStd

    unitTestSim.InitializeSimulation()
    for i in range(num_steps):
        stMessage.timeTag = time[i]
        stMessage.MRP_BdyInrtl = (truth[i, 1:4] + np.random.normal(0, stSigma, 3)).tolist()
        gyroBuffer.accPkts[0].gyro_B = (truthRate + np.random.normal(0, gyroSigma, 3)).tolist()
        if i > 5:
            stInMsg.write(stMessage, macros.sec2nano(time[i]))
            gyroInMsg.write(gyroBuffer, macros.sec2nano(time[i]))
        unitTestSim.ConfigureStopTime(macros.sec2nano(time[i + 1]))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.state[:, :NUM_STATES])
    covarLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.covar[:, :NUM_STATES ** 2])

    # Star-tracker and gyro residuals (3 components each), pre- and post-fit.
    st_pre = unitTestSupport.addTimeColumn(stResLog.times(), stResLog.preFits[:, :3])
    st_post = unitTestSupport.addTimeColumn(stResLog.times(), stResLog.postFits[:, :3])
    gyro_pre = unitTestSupport.addTimeColumn(gyroResLog.times(), gyroResLog.preFits[:, :3])
    gyro_post = unitTestSupport.addTimeColumn(gyroResLog.times(), gyroResLog.postFits[:, :3])

    # Rate estimate must converge to the gyro rate.
    for i in range(3):
        if abs(stateLog[-1, 4 + i] - truthRate[i]) > 5 * gyroSigma:
            testFailCount += 1
            testMessages.append("Rate estimation failure")
    # Covariance diagonal must shrink below its initial value.
    for i in range(NUM_STATES):
        if covarLog[-1, i * NUM_STATES + i + 1] > covarLog[0, i * NUM_STATES + i + 1]:
            testFailCount += 1
            testMessages.append("Covariance shrink failure")

    # Once converged, the post-fit residuals must settle to within the measurement
    # noise: their RMS over the second half of the run stays inside the 10-sigma band
    # (a generous bound avoids false positives while still confirming convergence).
    half = len(st_post) // 2
    st_valid = np.array(stResLog.valid, dtype=bool)[half:]
    gyro_valid = np.array(gyroResLog.valid, dtype=bool)[half:]
    st_post_rms = np.sqrt(np.mean(st_post[half:, 1:4][st_valid] ** 2, axis=0))
    gyro_post_rms = np.sqrt(np.mean(gyro_post[half:, 1:4][gyro_valid] ** 2, axis=0))
    if np.any(st_post_rms > 10 * stSigma):
        testFailCount += 1
        testMessages.append("Star-tracker post-fit residuals exceed 10-sigma noise: " + str(st_post_rms))
    if np.any(gyro_post_rms > 10 * gyroSigma):
        testFailCount += 1
        testMessages.append("Gyro post-fit residuals exceed 10-sigma noise: " + str(gyro_post_rms))

    if show_plots:
        diff = np.copy(stateLog)
        diff[:, 1:] -= truth[:len(diff), 1:]
        filter_plots.state_covar(stateLog, covarLog, 'Rate Update').show()
        filter_plots.states(diff, 'Rate Update').show()
        filter_plots.post_fit_residuals(st_pre, stSigma, 'Star Tracker Pre-Fit').show()
        filter_plots.post_fit_residuals(st_post, stSigma, 'Star Tracker Post-Fit').show()
        filter_plots.post_fit_residuals(gyro_pre, gyroSigma, 'Gyro Pre-Fit').show()
        filter_plots.post_fit_residuals(gyro_post, gyroSigma, 'Gyro Post-Fit').show()

    if testFailCount == 0:
        print("PASSED: InertialSRuKF rate update")
    return [testFailCount, ''.join(testMessages)]


def test_statePropagation(show_plots):
    [testResults, testMessage] = statePropagation(show_plots)
    assert testResults < 1, testMessage


def statePropagation(show_plots):
    """With no measurements and zero initial rate, the attitude stays fixed and the
    covariance grows under the process noise."""
    testFailCount = 0
    testMessages = []

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    dt = 1.0
    testProcessRate = macros.sec2nano(dt)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = inertialSRuKFF32.InertialSRuKF()
    unitTestSim.AddModelToTask(unitTaskName, module)
    setupFilterData(module)
    module.initialState = [0.1, 0.2, -0.1, 0.0, 0.0, 0.0]

    filterLog = module.filterOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, filterLog)

    # Connect a star-tracker input but never give it a fresh time tag, so no measurement
    # fires and the filter only propagates.
    stMessage = messaging.STAttMsgPayload()
    stMessage.timeTag = -1
    stInMsg = messaging.STAttMsg().write(stMessage)
    module.stAttInMsg.subscribeTo(stInMsg)

    initState = np.array(module.initialState)

    unitTestSim.InitializeSimulation()
    sim_time = 50
    unitTestSim.ConfigureStopTime(macros.sec2nano(sim_time))
    unitTestSim.ExecuteSimulation()

    stateLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.state[:, :NUM_STATES])
    covarLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.covar[:, :NUM_STATES ** 2])

    accuracy = 1.0E-6
    for i in range(NUM_STATES):
        if abs(stateLog[-1, i + 1] - initState[i]) > accuracy:
            testFailCount += 1
            testMessages.append("State propagation failure")
    if np.linalg.norm(covarLog[-1, 1:]) <= np.linalg.norm(covarLog[0, 1:]):
        testFailCount += 1
        testMessages.append("Covariance growth failure")

    if testFailCount == 0:
        print("PASSED: InertialSRuKF state propagation")
    return [testFailCount, ''.join(testMessages)]


def test_delayedMeasurement(show_plots):
    [testResults, testMessage] = delayedMeasurement(show_plots)
    assert testResults < 1, testMessage


def delayedMeasurement(show_plots):
    """Exercise the queue/anchor time bookkeeping across three configurations:

    1. Converge with a fresh star-tracker measurement every step.
    2. Let measurements be absent for several steps: the filter time-updates (propagates)
       while the last-measurement anchor stays put and the covariance grows.
    3. Deliver a measurement stamped at a time that is *in the past* relative to the
       current call time but still *newer than the anchor*. The filter must accept it
       (move the anchor to the measurement time, apply the update there, then propagate
       the remainder forward to the call time).
    4. Delivering that measurement late yields the same final estimate and covariance as
       delivering it on time, because the filter anchors to the measurement's time-tag
       rather than to the delivery time.

    The discriminating check is that the delayed measurement's residual is valid: if the
    anchor had been dragged forward by the gap's propagation, a measurement time-stamped
    before the call time would be dropped instead of applied.

    Star-tracker only (no gyro): the body rate stays at its initial value, so the attitude
    propagates along the known truth and the propagation is directly observable."""
    testFailCount = 0
    testMessages = []

    dt = 0.5
    num_steps = 120
    n_converge = 60   # phase 1: a fresh measurement every step
    n_gap_end = 100   # phase 2: no fresh measurement on steps [n_converge, n_gap_end)
    meas_idx = 80     # the post-convergence measurement is stamped at this gap time

    # Known truth: constant body rate, attitude integrated from the MRP kinematics.
    truthRate = np.array([0.01, -0.005, 0.008])
    time = np.linspace(0, num_steps * dt, num_steps + 1)
    truth = rk4(mrp_integration, time, np.array([0.0, 0.0, 0.0, *truthRate.tolist()]), mrpShadow=True)

    def run_scenario(deliver_step):
        """Run phase-1 convergence, then deliver the single time[meas_idx] star-tracker
        measurement at step `deliver_step` (always stamped at time[meas_idx]). Star-tracker
        only (no gyro): the body rate stays at its initial value, so the attitude propagates
        along the known truth. Returns (stateLog, covarLog, st_valid)."""
        unitTestSim = SimulationBaseClass.SimBaseClass()
        testProc = unitTestSim.CreateNewProcess("TestProcess")
        testProc.addTask(unitTestSim.CreateNewTask("unitTask", macros.sec2nano(dt)))

        module = inertialSRuKFF32.InertialSRuKF()
        unitTestSim.AddModelToTask("unitTask", module)
        setupFilterData(module)
        # Start with an attitude error (and the truth rate) so measurements have something
        # to correct.
        module.initialState = [0.05, -0.03, 0.04, *truthRate.tolist()]

        filterLog = module.filterOutMsg.recorder()
        unitTestSim.AddModelToTask("unitTask", filterLog)
        stResLog = module.filterStResOutMsg.recorder()
        unitTestSim.AddModelToTask("unitTask", stResLog)

        stMessage = messaging.STAttMsgPayload()
        stInMsg = messaging.STAttMsg()
        module.stAttInMsg.subscribeTo(stInMsg)

        unitTestSim.InitializeSimulation()
        for i in range(num_steps):
            if i < n_converge:
                stMessage.timeTag = time[i]
                stMessage.MRP_BdyInrtl = truth[i, 1:4].tolist()
                stInMsg.write(stMessage, macros.sec2nano(time[i]))
            elif i == deliver_step:
                # Always stamped at time[meas_idx], regardless of when it is delivered.
                stMessage.timeTag = time[meas_idx]
                stMessage.MRP_BdyInrtl = truth[meas_idx, 1:4].tolist()
                stInMsg.write(stMessage, macros.sec2nano(time[meas_idx]))
            # Otherwise write nothing -> no fresh measurement that step.
            unitTestSim.ConfigureStopTime(macros.sec2nano(time[i + 1]))
            unitTestSim.ExecuteSimulation()

        stateLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.state[:, :NUM_STATES])
        covarLog = unitTestSupport.addTimeColumn(filterLog.times(), filterLog.covar[:, :NUM_STATES ** 2])
        st_valid = np.array(stResLog.valid, dtype=bool)
        return stateLog, covarLog, st_valid

    # Delayed: the measurement is delivered late, at n_gap_end. On-time reference: it is
    # delivered at its own time step (meas_idx).
    stateLog, covarLog, st_valid = run_scenario(n_gap_end)
    refState, refCovar, _ = run_scenario(meas_idx)

    def attitude_trace(cov_row):
        return float(sum(cov_row[1 + i * NUM_STATES + i] for i in range(3)))

    # Log row k corresponds to sim time[k + 1] (the first execute advances to time[1]).

    # Phase 1: the estimate converges to the truth.
    if np.linalg.norm(stateLog[n_converge - 1, 1:4] - truth[n_converge, 1:4]) > 1.0E-2:
        testFailCount += 1
        testMessages.append("phase 1 did not converge")

    # Phase 2: no measurement fires during the gap, the estimate keeps propagating along
    # the truth, and the covariance grows.
    if st_valid[n_converge:n_gap_end].any():
        testFailCount += 1
        testMessages.append("a measurement fired during the no-measurement gap")
    for k in range(n_converge, n_gap_end):
        if np.linalg.norm(stateLog[k, 1:4] - truth[k + 1, 1:4]) > 1.0E-2:
            testFailCount += 1
            testMessages.append("propagation drifted off truth during the gap")
            break
    if attitude_trace(covarLog[n_gap_end - 1]) <= attitude_trace(covarLog[n_converge - 1]):
        testFailCount += 1
        testMessages.append("covariance did not grow across the measurement gap")

    # Phase 3: the past-but-new measurement is accepted (its residual is valid), the
    # estimate is propagated to the call time, and the attitude covariance shrinks again.
    if not st_valid[n_gap_end]:
        testFailCount += 1
        testMessages.append("past-but-new measurement was dropped (anchor moved with propagation)")
    if np.linalg.norm(stateLog[n_gap_end, 1:4] - truth[n_gap_end + 1, 1:4]) > 1.0E-2:
        testFailCount += 1
        testMessages.append("estimate not propagated to the call time after the delayed measurement")
    if attitude_trace(covarLog[n_gap_end]) >= attitude_trace(covarLog[n_gap_end - 1]):
        testFailCount += 1
        testMessages.append("delayed measurement did not shrink the covariance")

    # Equivalence: because the filter anchors to the measurement's time-tag, delivering it
    # late produces the same final estimate and covariance as delivering it on time.
    if not np.allclose(stateLog[-1, 1:], refState[-1, 1:], rtol=0, atol=1.0E-9):
        testFailCount += 1
        testMessages.append("delayed-measurement final state differs from the on-time result")
    if not np.allclose(covarLog[-1, 1:], refCovar[-1, 1:], rtol=0, atol=1.0E-9):
        testFailCount += 1
        testMessages.append("delayed-measurement final covariance differs from the on-time result")

    if show_plots:
        filter_plots.state_covar(stateLog, covarLog, 'Delayed Measurement').show()

    if testFailCount == 0:
        print("PASSED: InertialSRuKF delayed measurement")
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    stateUpdateInertialAttitude(True)
    stateUpdateRate(True)
    statePropagation(False)
