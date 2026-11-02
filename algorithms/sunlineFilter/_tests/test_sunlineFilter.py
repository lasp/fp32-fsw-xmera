import sunlineFilter_test_utilities as filter_plots
import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import sunlineFilterF32
from xmera.utilities import SimulationBaseClass, macros
from xmera.utilities import RigidBodyKinematics as rbk


def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))


def rk4(f, t, x0, normalizeState=False, mrpShadow=False):
    x = np.zeros([len(t), len(x0) + 1])
    h = (t[len(t) - 1] - t[0]) / len(t)
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
        if normalizeState:
            # Normalize the states and bound the bias state to the default values
            x[i + 1, 1:4] = x[i + 1, 1:4] / np.linalg.norm(x[i + 1, 1:4])
            if x[i + 1, 7] < 0.5:
                x[i + 1, 7] = 0.5
            if x[i + 1, 7] > 1.5:
                x[i + 1, 7] = 1.5
        if mrpShadow:
            s = np.linalg.norm(x[i + 1, 1:4])**2
            if s > 1:
                x[i + 1, 1:4] = - (x[i + 1, 1:4]) / s
        x[i + 1, 0] = t[i + 1]
    return x


def sunline_dynamics(t, x):
    dxdt = np.zeros(np.shape(x))
    dxdt[0:3] = np.cross(x[:3], x[3:6])
    dxdt[3:7] = np.zeros(4)
    return dxdt


def mrp_integration(t, x):
    dxdt = np.zeros(np.shape(x))
    B = rbk.BmatMRP(x[0:3])
    dxdt[:3] = 0.25 * np.matmul(B, x[3:6])
    dxdt[3:6] = np.zeros(3)
    return dxdt


def setup_filter_data(filter_object):
    filter_object.alpha = 0.02
    filter_object.beta = 2.0

    filter_object.initialState = [0.0, 0.0, 1.0, 0.02, -0.005, 0.01, 0.6]
    filter_object.initialCovariance = [[0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                       [0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0],
                                       [0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0],
                                       [0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0],
                                       [0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0],
                                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0],
                                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1]]

    filter_object.cssMeasurementNoiseStd = 0.0001
    filter_object.gyroMeasurementNoiseStd = 0.00001
    sigmaSun = (1E-5) ** 2
    sigmaRate = (1E-5) ** 2
    sigmaBias = (1E-5) ** 2
    filter_object.processNoise = [[sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigmaBias]]

def setup_css_config_msg(CSSOrientationList, cssConfigDataInMsg):
    numCSS = len(CSSOrientationList)

    # set the CSS unit vectors
    cssConfigData = messaging.CSSConfigMsgF32Payload()
    totalCSSList = []
    for CSSHat in CSSOrientationList:
        CSSConfigElement = messaging.CSSUnitConfigMsgF32Payload()
        CSSConfigElement.CBias = 1.0
        CSSConfigElement.nHat_B = CSSHat
        totalCSSList.append(CSSConfigElement)
    cssConfigData.nCSS = numCSS
    cssConfigData.cssVals = totalCSSList
    cssConfigDataInMsg.write(cssConfigData)


def test_propagation_kf(show_plots):
    state_propagation_flyby(show_plots)

@pytest.mark.parametrize("initial_error", [False, True])
def test_measurements_kf(show_plots, initial_error):
    state_update_flyby(initial_error, False)


def state_propagation_flyby(show_plots=False):
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(1.0)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    sunHeadingFilter = sunlineFilterF32.SunlineFilter()

    # Add test module to runtime call list
    setup_filter_data(sunHeadingFilter)
    unit_test_sim.AddModelToTask(unit_task_name, sunHeadingFilter)

    sun_heading_data_log = sunHeadingFilter.filterOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, sun_heading_data_log)

    simpleNavMsgData = messaging.NavAttMsgF32Payload()
    initState = np.array(sunHeadingFilter.initialState).reshape(7)
    simpleNavMsgData.timeTag = -1
    simpleNavMsgData.omega_BN_B = initState[3:6]
    simpleNavMsg = messaging.NavAttMsgF32().write(simpleNavMsgData)
    sunHeadingFilter.navAttInMsg.subscribeTo(simpleNavMsg)

    CSSOrientationList = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0],
    ]

    cssConfigMsg = messaging.CSSConfigMsgF32()
    setup_css_config_msg(CSSOrientationList, cssConfigMsg)
    sunHeadingFilter.cssConfigInMsg.subscribeTo(cssConfigMsg)

    cssDataMsg = messaging.CSSArraySensorMsgF32Payload()
    cssDataMsg.timeTag = -1
    for i in range(8):
        cssDataMsg.CosValue[i] = 0.0
    cssMsg = messaging.CSSArraySensorMsgF32().write(cssDataMsg)
    sunHeadingFilter.cssDataInMsg.subscribeTo(cssMsg)

    sim_time = 50
    time = np.linspace(0, sim_time, sim_time+1)
    expected = np.zeros([len(time), 8])
    expected[0, 1:] = initState
    expected = rk4(sunline_dynamics, time, expected[0, 1:], normalizeState=True)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    num_states = 7
    state_data_log = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.covar[:, :num_states**2])

    np.testing.assert_array_less(np.linalg.norm(covariance_data_log[0, 1:]),
                                 np.linalg.norm(covariance_data_log[-1, 1:]),
                                 err_msg='covariance must increase without measurements',
                                 verbose=True)
    np.testing.assert_allclose(state_data_log[:, 1:],
                               expected[:, 1:],
                               rtol=1E-8,
                               atol=3E-5,
                               err_msg='state propagation error',
                               verbose=True)
    diff = np.copy(state_data_log)
    diff[:, 1:] -= expected[:, 1:]
    if show_plots:
        filter_plots.state_covar(state_data_log, covariance_data_log, 'Update').show()
        filter_plots.states(diff, 'Update').show()

def state_update_flyby(initial_error, show_plots=False):
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(1.0)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    sunHeadingFilter = sunlineFilterF32.SunlineFilter()

    # Add test module to runtime call list
    setup_filter_data(sunHeadingFilter)
    unit_test_sim.AddModelToTask(unit_task_name, sunHeadingFilter)

    sun_heading_data_log = sunHeadingFilter.filterOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, sun_heading_data_log)

    css_residual_data_log = sunHeadingFilter.filterCssResOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, css_residual_data_log)

    gyro_residual_data_log = sunHeadingFilter.filterGyroResOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, gyro_residual_data_log)

    nav_att_data_log = sunHeadingFilter.navAttOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, nav_att_data_log)

    simpleNavMsgData = messaging.NavAttMsgF32Payload()
    initState = np.array(sunHeadingFilter.initialState).reshape(7)
    simpleNavMsgData.timeTag = -1
    simpleNavMsgData.omega_BN_B = initState[3:6]
    simpleNavMsg = messaging.NavAttMsgF32().write(simpleNavMsgData)
    sunHeadingFilter.navAttInMsg.subscribeTo(simpleNavMsg)

    CSSOrientationList = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0],
    ]

    cssConfigMsg = messaging.CSSConfigMsgF32()
    setup_css_config_msg(CSSOrientationList, cssConfigMsg)
    sunHeadingFilter.cssConfigInMsg.subscribeTo(cssConfigMsg)

    sim_time = 2000
    np.random.seed(0)
    time = np.linspace(0, sim_time, sim_time+1)
    expected = np.zeros([len(time), 8])
    expected[0, 1:] = initState
    expected = rk4(sunline_dynamics, time, expected[0, 1:], normalizeState=True)

    bodyFrame = np.zeros([len(time), 8])
    bodyFrame[0, 1:] = np.array([0.0, 0.0, 0.0, expected[0, 4], expected[0, 5], expected[0, 6], expected[0, 7]])
    bodyFrame = rk4(mrp_integration, time, bodyFrame[0, 1:], mrpShadow=True)

    if initial_error:
        sunHeadingFilter.initialState = [1.0, 0.0, 0.0, -0.02, 0.005, -0.01, 1]
        sunHeadingFilter.initialCovariance = [[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                              [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0],
                                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0],
                                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]]

    cssDataMsg = messaging.CSSArraySensorMsgF32Payload()
    cssMsg = messaging.CSSArraySensorMsgF32()
    sunHeadingFilter.cssDataInMsg.subscribeTo(cssMsg)

    cssSigma = sunHeadingFilter.cssMeasurementNoiseStd
    gyroSigma = sunHeadingFilter.gyroMeasurementNoiseStd
    unit_test_sim.InitializeSimulation()
    for i in range(0, len(time)-1):
        BN = rbk.MRP2C(bodyFrame[i, 1:4])
        cosList = []
        for j in range(len(CSSOrientationList)):
            cosList.append((np.dot(CSSOrientationList[j], np.matmul(BN, [0, 0, 1]))
                            + np.random.normal(0, cssSigma, 1))[0])
        cssDataMsg.CosValue = np.array(cosList)*expected[i, 7]
        cssDataMsg.timeTag = time[i]
        omega = expected[0, 4:7] + np.random.normal(0, gyroSigma, 3)
        simpleNavMsgData.timeTag = time[i]
        simpleNavMsgData.omega_BN_B = omega
        if i % 2 == 0:
            cssMsg.write(cssDataMsg)
            simpleNavMsg.write(simpleNavMsgData)
        unit_test_sim.ConfigureStopTime(macros.sec2nano(time[i+1]))
        unit_test_sim.ExecuteSimulation()

    num_states = 7
    state_data_log = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.covar[:, :num_states**2])

    covariance = []
    for i in range(num_states):
        covariance.append([])
        for j in range(num_states):
            covariance[-1].append(covariance_data_log[i][1+j*(num_states+1)])

    css_number_obs = css_residual_data_log.numberOfObservations
    css_size_obs = css_residual_data_log.sizeOfObservations
    css_post_fit_log_sparse = add_time_column(css_residual_data_log.times(), css_residual_data_log.postFits)
    css_post_fit_log = np.zeros([len(css_residual_data_log.times()), len(CSSOrientationList) + 1])
    css_post_fit_log[:, 0] = css_post_fit_log_sparse[:, 0]
    css_pre_fit_log_sparse = add_time_column(css_residual_data_log.times(), css_residual_data_log.preFits)
    css_pre_fit_log = np.zeros([len(css_residual_data_log.times()), len(CSSOrientationList) + 1])
    css_pre_fit_log[:, 0] = css_pre_fit_log_sparse[:, 0]

    for i in range(len(css_number_obs)):
        if css_number_obs[i] > 0:
            css_post_fit_log[i, 1:css_size_obs[i]+1] = css_post_fit_log_sparse[i, 1:css_size_obs[i]+1]
            css_pre_fit_log[i, 1:css_size_obs[i]+1] = css_pre_fit_log_sparse[i, 1:css_size_obs[i]+1]

    gyro_number_obs = gyro_residual_data_log.numberOfObservations
    gyro_size_obs = gyro_residual_data_log.sizeOfObservations
    gyro_post_fit_log_sparse = add_time_column(gyro_residual_data_log.times(), gyro_residual_data_log.postFits)
    gyro_post_fit_log = np.zeros([len(gyro_residual_data_log.times()), np.max(gyro_size_obs)+1])
    gyro_post_fit_log[:, 0] = gyro_post_fit_log_sparse[:, 0]
    gyro_pre_fit_log_sparse = add_time_column(gyro_residual_data_log.times(), gyro_residual_data_log.preFits)
    gyro_pre_fit_log = np.zeros([len(gyro_residual_data_log.times()), np.max(gyro_size_obs)+1])
    gyro_pre_fit_log[:, 0] = gyro_pre_fit_log_sparse[:, 0]

    for i in range(len(gyro_number_obs)):
        if gyro_number_obs[i] > 0:
            gyro_post_fit_log[i, 1:gyro_size_obs[i]+1] = gyro_post_fit_log_sparse[i, 1:gyro_size_obs[i]+1]
            gyro_pre_fit_log[i, 1:gyro_size_obs[i]+1] = gyro_pre_fit_log_sparse[i, 1:gyro_size_obs[i]+1]

    # Plot before asserting: when an assertion below fails, these figures are what explains it.
    diff = np.copy(state_data_log)
    diff[:, 1:] -= expected[:, 1:]
    if show_plots:
        filter_plots.state_covar(state_data_log, covariance_data_log, 'Update').show()
        filter_plots.states(diff, 'Update').show()

    half_time = len(time) // 2
    # testing that Sun Heading vector estimate is correct within 5 sigma
    np.testing.assert_allclose(state_data_log[half_time:, 1:4],
                               expected[half_time:, 1:4],
                                atol=5*cssSigma,
                                err_msg='heading estimation error',
                                verbose=True)
    # testing that rate estimate is correct within 5 sigma
    np.testing.assert_allclose(state_data_log[half_time:, 4:7],
                                expected[half_time:, 4:7],
                               atol=5*gyroSigma,
                               err_msg='rate estimation error',
                               verbose=True)
    # testing that rate estimate is correct within 5 sigma
    np.testing.assert_allclose(state_data_log[half_time:, 7],
                                expected[half_time:, 7],
                               atol=0.2,
                               err_msg='bias estimation error',
                               verbose=True)
    # testing that covariance is shrinking
    np.testing.assert_array_less(np.diag(covariance_data_log[half_time, 1:7*7+1].reshape([7, 7])),
                                np.diag(covariance_data_log[0, 1:7*7+1].reshape([7, 7])),
                                err_msg='covariance error',
                                verbose=True)

    diff = np.copy(state_data_log)
    diff[:, 1:] -= expected[:, 1:]
    if show_plots:
        filter_plots.state_covar(state_data_log, covariance_data_log, 'Update').show()
        filter_plots.states(diff, 'Update').show()
        filter_plots.post_fit_residuals(css_post_fit_log, cssSigma, 'Update CSS PreFit').show()
        filter_plots.post_fit_residuals(css_pre_fit_log, cssSigma, 'Update CSS PostFit').show()
        filter_plots.post_fit_residuals(gyro_post_fit_log, gyroSigma, 'Update Gyro PreFit').show()
        filter_plots.post_fit_residuals(gyro_pre_fit_log, gyroSigma, 'Update Gyro PostFit').show()


# Steps at which a gross outlier is mixed into the measurement stream. The two sensors are kept on
# disjoint steps so each stream's perturbation is attributable to it alone.
CSS_OUTLIER_STEPS = (100, 200, 300, 400, 500)
GYRO_OUTLIER_STEPS = (150, 250, 350, 450)
CSS_OUTLIER = 3.0                            # [-] cosine offset added to the brightest sensor
GYRO_OUTLIER = np.array([0.5, 0.5, -0.5])    # [rad/s] offset added to a gyro sample


def test_outlier_recovery_kf(show_plots):
    state_update_outlier_recovery(show_plots)


def state_update_outlier_recovery(show_plots=False):
    """Track the sun-heading profile with occasional gross CSS and gyro outliers, and check the
    filter has converged by the last step."""
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    dt = 1.0
    test_process_rate = macros.sec2nano(dt)
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    sunHeadingFilter = sunlineFilterF32.SunlineFilter()
    setup_filter_data(sunHeadingFilter)
    unit_test_sim.AddModelToTask(unit_task_name, sunHeadingFilter)

    sun_heading_data_log = sunHeadingFilter.filterOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, sun_heading_data_log)
    css_residual_data_log = sunHeadingFilter.filterCssResOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, css_residual_data_log)

    simpleNavMsgData = messaging.NavAttMsgF32Payload()
    initState = np.array(sunHeadingFilter.initialState).reshape(7)
    simpleNavMsgData.timeTag = -1
    simpleNavMsgData.omega_BN_B = initState[3:6]
    simpleNavMsg = messaging.NavAttMsgF32().write(simpleNavMsgData)
    sunHeadingFilter.navAttInMsg.subscribeTo(simpleNavMsg)

    CSSOrientationList = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0],
    ]
    num_css = len(CSSOrientationList)

    cssConfigMsg = messaging.CSSConfigMsgF32()
    setup_css_config_msg(CSSOrientationList, cssConfigMsg)
    sunHeadingFilter.cssConfigInMsg.subscribeTo(cssConfigMsg)

    num_steps = 600
    np.random.seed(0)
    time = np.linspace(0, num_steps, num_steps + 1)
    expected = np.zeros([len(time), 8])
    expected[0, 1:] = initState
    expected = rk4(sunline_dynamics, time, expected[0, 1:], normalizeState=True)

    bodyFrame = np.zeros([len(time), 8])
    bodyFrame[0, 1:] = np.array([0.0, 0.0, 0.0, expected[0, 4], expected[0, 5], expected[0, 6], expected[0, 7]])
    bodyFrame = rk4(mrp_integration, time, bodyFrame[0, 1:], mrpShadow=True)

    cssDataMsg = messaging.CSSArraySensorMsgF32Payload()
    cssMsg = messaging.CSSArraySensorMsgF32()
    sunHeadingFilter.cssDataInMsg.subscribeTo(cssMsg)

    cssSigma = sunHeadingFilter.cssMeasurementNoiseStd
    gyroSigma = sunHeadingFilter.gyroMeasurementNoiseStd
    # The gyro is stamped half a step after the CSS array so the queue order within a cycle is fixed
    # and the two streams cannot race.
    gyro_time_offset = 0.5 * dt

    css_meas = np.zeros([num_steps, num_css + 1])
    css_truth = np.zeros([num_steps, num_css + 1])
    gyro_meas = np.zeros([num_steps, 4])
    gyro_truth = np.zeros([num_steps, 4])

    unit_test_sim.InitializeSimulation()
    for i in range(num_steps):
        BN = rbk.MRP2C(bodyFrame[i, 1:4])
        sunHeading_B = np.matmul(BN, [0, 0, 1])
        clean = np.array([np.dot(CSSOrientationList[j], sunHeading_B) for j in range(num_css)])
        cosValues = (clean + np.random.normal(0, cssSigma, num_css)) * expected[i, 7]
        omega = expected[0, 4:7] + np.random.normal(0, gyroSigma, 3)
        if i in CSS_OUTLIER_STEPS:
            # Corrupt the sensor that currently sees the most light, so the glitched reading is
            # certain to clear the sensor threshold and stay in the active set.
            cosValues[np.argmax(cosValues)] += CSS_OUTLIER
        if i in GYRO_OUTLIER_STEPS:
            omega = omega + GYRO_OUTLIER

        css_meas[i] = [macros.sec2nano(time[i]), *cosValues]
        css_truth[i] = [macros.sec2nano(time[i]), *(clean * expected[i, 7])]
        gyro_meas[i] = [macros.sec2nano(time[i]), *omega]
        gyro_truth[i] = [macros.sec2nano(time[i]), *expected[0, 4:7]]

        cssDataMsg.CosValue = cosValues
        cssDataMsg.timeTag = time[i]
        cssMsg.write(cssDataMsg)
        simpleNavMsgData.timeTag = time[i] + gyro_time_offset
        simpleNavMsgData.omega_BN_B = omega
        simpleNavMsg.write(simpleNavMsgData)

        unit_test_sim.ConfigureStopTime(macros.sec2nano(time[i + 1]))
        unit_test_sim.ExecuteSimulation()

    num_states = 7
    state_all = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.state[:, :num_states])
    covar_all = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.covar[:, :num_states ** 2])
    # Log row 0 is the seeded estimate at t = 0; from there on row k + 1 is loop step k, so drop the
    # first row to line the logs up with the injected-outlier step indices.
    state_data_log = state_all[1:]
    covariance_data_log = covar_all[1:]

    np.testing.assert_equal(len(state_data_log), num_steps,
                            err_msg='log length does not match the step count')

    heading_err = np.linalg.norm(state_data_log[:, 1:4] - expected[1:num_steps + 1, 1:4], axis=1)
    rate_err = np.linalg.norm(state_data_log[:, 4:7] - expected[1:num_steps + 1, 4:7], axis=1)
    # Reference level for the error_recovery figures: the accuracy before any outlier lands.
    heading_baseline = float(np.median(heading_err[30:min(CSS_OUTLIER_STEPS)]))
    rate_baseline = float(np.median(rate_err[30:min(GYRO_OUTLIER_STEPS)]))

    if show_plots:
        diff = np.copy(state_data_log)
        diff[:, 1:] -= expected[1:num_steps + 1, 1:]
        used = np.ones(num_steps, dtype=bool)
        heading_err_col = np.column_stack([state_data_log[:, 0], heading_err])
        rate_err_col = np.column_stack([state_data_log[:, 0], rate_err])
        filter_plots.outlier_rejection(css_meas, css_truth, used, 'CSS Outliers').show()
        filter_plots.outlier_rejection(gyro_meas, gyro_truth, used, 'Gyro Outliers').show()
        filter_plots.error_recovery(heading_err_col, CSS_OUTLIER_STEPS, heading_baseline, 'Heading').show()
        filter_plots.error_recovery(rate_err_col, GYRO_OUTLIER_STEPS, rate_baseline, 'Rate').show()
        filter_plots.state_covar(state_data_log, covariance_data_log, 'Outlier Recovery').show()
        filter_plots.states(diff, 'Outlier Recovery').show()

    # Tolerances are multiples of the measurement noise so they follow the tuning; covariance is
    # checked on the trace because an individual block can end above its seeded value.
    np.testing.assert_allclose(state_data_log[-1, 1:4], expected[-1, 1:4], atol=5 * cssSigma,
                               err_msg='heading not converged at the last step', verbose=True)
    np.testing.assert_allclose(state_data_log[-1, 4:7], expected[-1, 4:7], atol=5 * gyroSigma,
                               err_msg='rate not converged at the last step', verbose=True)
    np.testing.assert_allclose(state_data_log[-1, 7], expected[-1, 7], atol=0.2,
                               err_msg='bias not converged at the last step', verbose=True)
    np.testing.assert_array_less(np.trace(covariance_data_log[-1, 1:num_states ** 2 + 1].reshape([num_states] * 2)),
                                 0.01 * np.trace(covar_all[0, 1:num_states ** 2 + 1].reshape([num_states] * 2)),
                                 err_msg='covariance not low at the last step', verbose=True)


# Timing of the delayed-gyro test, sized for the sunline BatchSize of 5. A reading with time tag k is written at loop
# step k and reaches the filter at call k + 1. The late gyro reading reaches it GYRO_EXTRA_STEPS = 2 calls later, at
# call k + 3, after the CSS reading k + 1 has moved the anchor past it. (One call late is not enough to drop it: it
# arrives with CSS reading k + 1 and the queue applies it first.) With MEASUREMENT_DELAY, reading k is due at call
# ceil(k + MEASUREMENT_DELAY) = k + 3, the call where the late gyro arrives. Before that call pops, the queue holds the
# CSS readings k .. k + 2 and the gyro reading k: 4 measurements. A gyro on time with the same delay would add the
# gyro readings k + 1 and k + 2 (6 measurements); the full queue would drop its oldest, due, reading.
GYRO_EXTRA_STEPS = 2      # [-] calls the late gyro reading arrives after the CSS reading of the same time tag
MEASUREMENT_DELAY = 2.5   # [s] processing delay of the delayed filter


def applied_time_tags(residual_log, readings, time_tags):
    """Identify the reading applied at each filter call from the observation in its residual message.

    Every reading carries its own noise, so its observation identifies it. A residual message reports only the last
    reading of its kind applied during the call.
    @return [call time (s), time tag (s)] of each call that applied a reading of this kind
    @param residual_log recorder of a filter residual message
    @param readings     [num_readings, n] float32 readings as the filter packs them (active CSS first)
    @param time_tags    [num_readings] time tag of each reading"""
    call_times = residual_log.times() * 1E-9
    applied = []
    for call_time, valid, size, observation in zip(call_times, residual_log.valid, residual_log.sizeOfObservations,
                                                   residual_log.observation):
        if not valid:
            continue
        mismatch = np.max(np.abs(readings[:, :size] - observation[:size]), axis=1)
        k = int(np.argmin(mismatch))
        assert mismatch[k] == 0.0, f'the observation applied at t = {call_time} s matches no reading'
        applied.append([call_time, time_tags[k]])
    return np.array(applied).reshape(-1, 2)


def test_delayed_gyro_kf(show_plots):
    state_update_delayed_gyro(show_plots)


def state_update_delayed_gyro(show_plots=False):
    """Run the filter with a gyro that lags behind the CSS, with and without a measurement delay.

    At step i the CSS and gyro readings sampled at time[i] are generated once. The filters run at time[i + 1], so
    a reading written at step i reaches them 1 s after its time tag. Three filters see the same readings:

    * gyro on time, no delay: the gyro is written with the CSS reading (latency 1 s) -- the normal cycle and the
      performance reference;
    * late gyro, no delay: the same gyro reading is written GYRO_EXTRA_STEPS later (latency 3 s). A newer CSS
      reading has already moved the anchor past it, so it is dropped;
    * late gyro, delayed: the late gyro with the delay. Every reading is applied once, in time-tag order, at the
      first call where it is due.
    """
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    dt = 1.0
    test_process_rate = macros.sec2nano(dt)
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    CSSOrientationList = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0],
    ]
    num_css = len(CSSOrientationList)
    cssConfigMsg = messaging.CSSConfigMsgF32()
    setup_css_config_msg(CSSOrientationList, cssConfigMsg)

    cssDataMsg = messaging.CSSArraySensorMsgF32Payload()
    cssMsg = messaging.CSSArraySensorMsgF32()
    gyro_msgs = {}
    for stream in ('on time', 'late'):
        payload = messaging.NavAttMsgF32Payload()
        payload.timeTag = -1
        gyro_msgs[stream] = messaging.NavAttMsgF32().write(payload)

    # label: (gyro stream, measurement delay)
    configurations = {
        'gyro on time, no delay': ('on time', 0.0),
        'late gyro, no delay': ('late', 0.0),
        'late gyro, delayed': ('late', MEASUREMENT_DELAY),
    }
    filters = {}
    for label, (stream, delay) in configurations.items():
        sunHeadingFilter = sunlineFilterF32.SunlineFilter()
        setup_filter_data(sunHeadingFilter)
        sunHeadingFilter.measurementDelay = delay
        sunHeadingFilter.navAttInMsg.subscribeTo(gyro_msgs[stream])
        sunHeadingFilter.cssDataInMsg.subscribeTo(cssMsg)
        sunHeadingFilter.cssConfigInMsg.subscribeTo(cssConfigMsg)
        unit_test_sim.AddModelToTask(unit_task_name, sunHeadingFilter)
        logs = {'state': sunHeadingFilter.filterOutMsg.recorder(),
                'css': sunHeadingFilter.filterCssResOutMsg.recorder(),
                'gyro': sunHeadingFilter.filterGyroResOutMsg.recorder()}
        for log in logs.values():
            unit_test_sim.AddModelToTask(unit_task_name, log)
        filters[label] = logs

    reference_filter = sunlineFilterF32.SunlineFilter()
    setup_filter_data(reference_filter)
    initState = np.array(reference_filter.initialState).reshape(7)
    cssSigma = reference_filter.cssMeasurementNoiseStd
    gyroSigma = reference_filter.gyroMeasurementNoiseStd
    sensorThreshold = reference_filter.sensorThreshold

    num_steps = 600
    np.random.seed(0)
    time = np.linspace(0, num_steps, num_steps + 1)
    expected = np.zeros([len(time), 8])
    expected[0, 1:] = initState
    expected = rk4(sunline_dynamics, time, expected[0, 1:], normalizeState=True)

    bodyFrame = np.zeros([len(time), 8])
    bodyFrame[0, 1:] = np.array([0.0, 0.0, 0.0, expected[0, 4], expected[0, 5], expected[0, 6], expected[0, 7]])
    bodyFrame = rk4(mrp_integration, time, bodyFrame[0, 1:], mrpShadow=True)

    # Gyro reading sampled at time[i]: the body rate is constant, so it is the truth rate plus noise. Generated
    # once so the on-time and late streams carry the same values and time tags.
    gyro_readings = expected[0, 4:7] + np.random.normal(0, gyroSigma, [num_steps, 3])
    # CSS readings as the filter packs them: float32, active sensors first. Used to identify the applied readings.
    css_packed = np.zeros([num_steps, num_css], dtype=np.float32)

    gyro_time_tags_sent = {'on time': [], 'late': []}

    def write_gyro(stream, k):
        payload = messaging.NavAttMsgF32Payload()
        payload.timeTag = time[k]
        payload.omega_BN_B = gyro_readings[k]
        gyro_msgs[stream].write(payload)
        gyro_time_tags_sent[stream].append(time[k])

    unit_test_sim.InitializeSimulation()
    for i in range(num_steps):
        BN = rbk.MRP2C(bodyFrame[i, 1:4])
        sunHeading_B = np.matmul(BN, [0, 0, 1])
        clean = np.array([np.dot(CSSOrientationList[j], sunHeading_B) for j in range(num_css)])
        cssDataMsg.CosValue = (clean + np.random.normal(0, cssSigma, num_css)) * expected[i, 7]
        cssDataMsg.timeTag = time[i]
        cssMsg.write(cssDataMsg)
        cos_values = np.array(cssDataMsg.CosValue[:num_css], dtype=np.float32)
        active = cos_values[cos_values > sensorThreshold]
        css_packed[i, :len(active)] = active

        if i > 0:  # a time tag of 0 is read as "no measurement"
            write_gyro('on time', i)
        if i - GYRO_EXTRA_STEPS > 0:
            write_gyro('late', i - GYRO_EXTRA_STEPS)

        unit_test_sim.ConfigureStopTime(macros.sec2nano(time[i + 1]))
        unit_test_sim.ExecuteSimulation()

    gyro_packed = gyro_readings.astype(np.float32)
    num_states = 7
    results = {}
    for label, logs in filters.items():
        state_log = logs['state']
        state_all = add_time_column(state_log.times(), state_log.state[:, :num_states])
        covar_all = add_time_column(state_log.times(), state_log.covar[:, :num_states ** 2])
        # Log row 0 is the seeded estimate at t = 0; row k + 1 is the output of loop step k, at time[k + 1].
        states = state_all[1:]
        heading_err = np.linalg.norm(states[:, 1:4] - expected[1:num_steps + 1, 1:4], axis=1)
        rate_err = np.linalg.norm(states[:, 4:7] - expected[1:num_steps + 1, 4:7], axis=1)
        results[label] = {
            'states': states,
            'covariance': covar_all[1:],
            'heading_err': np.column_stack([states[:, 0], heading_err]),
            'rate_err': np.column_stack([states[:, 0], rate_err]),
            'css_applied': applied_time_tags(logs['css'], css_packed, time[:num_steps]),
            'gyro_applied': applied_time_tags(logs['gyro'], gyro_packed, time[:num_steps]),
        }

    reference = results['gyro on time, no delay']
    undelayed = results['late gyro, no delay']
    delayed = results['late gyro, delayed']

    if show_plots:
        filter_plots.delay_comparison({label: r['rate_err'] for label, r in results.items()},
                                      '|rate error| (rad/s)', 'Late gyro: rate error').show()
        filter_plots.delay_comparison({label: r['heading_err'] for label, r in results.items()},
                                      '|heading error|', 'Late gyro: heading error').show()
        filter_plots.state_covar(delayed['states'], delayed['covariance'], 'Late gyro, delayed').show()

    # Order. With the delay, a reading with time tag t is due at the first call at or after t + MEASUREMENT_DELAY.
    # The late gyro has arrived by then, so every reading must be applied once, in time-tag order, at that call.
    final_time = time[num_steps]
    # The CSS reading at t = 0 is never enqueued: a time tag of 0 is read as "no measurement".
    for kind, sent_tags in (('css', time[1:num_steps]), ('gyro', gyro_time_tags_sent['late'])):
        applied = delayed[f'{kind}_applied']
        due_tags = np.array([t for t in sent_tags if t <= final_time - MEASUREMENT_DELAY])
        assert np.all(np.diff(applied[:, 1]) > 0), f'{kind} readings applied out of time-tag order'
        np.testing.assert_array_equal(applied[:, 1], due_tags,
                                      err_msg=f'{kind}: the applied readings are not exactly the due readings')
        np.testing.assert_allclose(applied[:, 0], np.ceil(applied[:, 1] + MEASUREMENT_DELAY), rtol=0, atol=1E-9,
                                   err_msg=f'{kind}: a reading was not applied at the first call where it is due')

    # Without the delay, each late gyro reading arrives behind the anchor set by a newer CSS reading and is dropped.
    np.testing.assert_array_equal(reference['gyro_applied'][:, 1], gyro_time_tags_sent['on time'],
                                  err_msg='every on-time gyro reading is applied without a delay')
    assert len(undelayed['gyro_applied']) == 0, 'without a delay, every late gyro reading is behind the anchor'

    # Performance against the normal cycle. With the delay, the filter applies the same readings, only later, so its
    # rate estimate is as good (measured ratio 1.01). Its output is a prediction over the delay, which costs some
    # heading accuracy (measured ratio 1.24). Without the delay, the rate comes from the CSS heading alone. Its error
    # is not checked: it is set by float rounding (measured ratios 1.1 to 330 against the reference across builds that
    # differ only in rounding), while the two filters checked here agree to three digits. The loss of the late gyro
    # readings is checked above, by the empty set of applied readings.
    second_half = slice(num_steps // 2, None)

    def rms_error(result, key):
        return float(np.sqrt(np.mean(result[key][second_half, 1] ** 2)))

    reference_rate = rms_error(reference, 'rate_err')
    reference_heading = rms_error(reference, 'heading_err')
    assert rms_error(delayed, 'rate_err') < 1.1 * reference_rate, \
        f"rate RMS with the delay {rms_error(delayed, 'rate_err')} vs reference {reference_rate}"
    assert rms_error(delayed, 'heading_err') < 1.6 * reference_heading, \
        f"heading RMS with the delay {rms_error(delayed, 'heading_err')} vs reference {reference_heading}"
    np.testing.assert_allclose(delayed['states'][-1, 4:7], expected[-1, 4:7], atol=5 * gyroSigma,
                               err_msg='rate not converged at the last step', verbose=True)
    np.testing.assert_allclose(delayed['states'][-1, 1:4], expected[-1, 1:4], atol=5 * cssSigma,
                               err_msg='heading not converged at the last step', verbose=True)



def test_properties_have_the_filter_size():
    """The matrix and vector properties have the filter's fixed size: a list of the wrong size is rejected."""
    module = sunlineFilterF32.SunlineFilter()
    n = 7
    np.testing.assert_array_equal(np.array(module.initialState).reshape(n), np.zeros(n))
    np.testing.assert_array_equal(module.initialCovariance, np.eye(n))
    np.testing.assert_array_equal(module.processNoise, np.zeros((n, n)))
    wrong_sizes = {
        "initialState": [0.0] * (n + 1),
        "initialCovariance": np.eye(n - 1).tolist(),
        "processNoise": np.eye(n + 1).tolist(),
    }
    for name, value in wrong_sizes.items():
        with pytest.raises(ValueError):
            setattr(module, name, value)


if __name__ == "__main__":
    state_update_flyby(True, True)
    state_update_outlier_recovery(True)
    state_update_delayed_gyro(True)
