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

    filter_object.cssMeasurementNoiseStd = 0.01
    filter_object.gyroMeasurementNoiseStd = 0.001
    sigmaSun = (1E-6) ** 2
    sigmaRate = (1E-8) ** 2
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
    cssConfigData = messaging.CSSConfigMsgPayload()
    totalCSSList = []
    for CSSHat in CSSOrientationList:
        CSSConfigElement = messaging.CSSUnitConfigMsgPayload()
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

    simpleNavMsgData = messaging.NavAttMsgPayload()
    initState = np.array(sunHeadingFilter.initialState).reshape(7)
    simpleNavMsgData.timeTag = -1
    simpleNavMsgData.omega_BN_B = initState[3:6]
    simpleNavMsg = messaging.NavAttMsg().write(simpleNavMsgData)
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

    cssConfigMsg = messaging.CSSConfigMsg()
    setup_css_config_msg(CSSOrientationList, cssConfigMsg)
    sunHeadingFilter.cssConfigInMsg.subscribeTo(cssConfigMsg)

    cssDataMsg = messaging.CSSArraySensorMsgPayload()
    cssDataMsg.timeTag = -1
    for i in range(8):
        cssDataMsg.CosValue[i] = 0.0
    cssMsg = messaging.CSSArraySensorMsg().write(cssDataMsg)
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

    simpleNavMsgData = messaging.NavAttMsgPayload()
    initState = np.array(sunHeadingFilter.initialState).reshape(7)
    simpleNavMsgData.timeTag = -1
    simpleNavMsgData.omega_BN_B = initState[3:6]
    simpleNavMsg = messaging.NavAttMsg().write(simpleNavMsgData)
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

    cssConfigMsg = messaging.CSSConfigMsg()
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
        sunHeadingFilter.initialCovariance = [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                              [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                              [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0],
                                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0],
                                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]]

    cssDataMsg = messaging.CSSArraySensorMsgPayload()
    cssMsg = messaging.CSSArraySensorMsg()
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
    """Track the normal sun-heading profile while occasional wildly wrong CSS and gyro samples are
    mixed in, and check the filter absorbs only a small part of each and settles back.

    A tuning instrument as much as a test: the fraction of an outlier that reaches the estimate is
    essentially the Kalman gain, which tracks sigma_Q/sigma_R, and the recovery length is how many
    measurements that gain needs to work the perturbation out. Lowering the ratio shrinks the first
    and lengthens the second. The printed table is the read-out for that trade.

    The assertions are upper bounds only, so they hold whether the filter screens outliers out
    before the update or simply out-weighs them."""
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

    simpleNavMsgData = messaging.NavAttMsgPayload()
    initState = np.array(sunHeadingFilter.initialState).reshape(7)
    simpleNavMsgData.timeTag = -1
    simpleNavMsgData.omega_BN_B = initState[3:6]
    simpleNavMsg = messaging.NavAttMsg().write(simpleNavMsgData)
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

    cssConfigMsg = messaging.CSSConfigMsg()
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

    cssDataMsg = messaging.CSSArraySensorMsgPayload()
    cssMsg = messaging.CSSArraySensorMsg()
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

    assert len(state_data_log) == num_steps, \
        'log length does not match the step count, outlier step indices are misaligned'

    heading_err = np.linalg.norm(state_data_log[:, 1:4] - expected[1:num_steps + 1, 1:4], axis=1)
    rate_err = np.linalg.norm(state_data_log[:, 4:7] - expected[1:num_steps + 1, 4:7], axis=1)
    outlier_steps = sorted(set(CSS_OUTLIER_STEPS) | set(GYRO_OUTLIER_STEPS))
    warmup = 30

    # The ambient level is measured away from the outliers, so a perturbation is judged against the
    # filter's own noise floor rather than an absolute number that goes stale on a retune.
    quiet = [k for k in range(warmup, num_steps) if all(abs(k - o) > 15 for o in outlier_steps)]
    heading_ambient = float(np.median(heading_err[quiet]))
    rate_ambient = float(np.median(rate_err[quiet]))

    def peak_and_recovery(err, step, ambient):
        peak = float(np.max(err[step:step + 3]))
        recovery = next((d for d in range(1, num_steps - step) if err[step + d] < 2 * ambient), None)
        return peak, recovery

    peak_bound = 10.0
    # Generous on purpose: the bound only catches a runaway, and the printed recovery figure is the
    # number worth watching. The first CSS outlier lands while the filter is still tightening, so its
    # recovery is dominated by the ongoing convergence rather than by the outlier (measured 42
    # measurements against 1 for every later one).
    recovery_bound = 100
    report = []
    for step in outlier_steps:
        is_css = step in CSS_OUTLIER_STEPS
        err, ambient = (heading_err, heading_ambient) if is_css else (rate_err, rate_ambient)
        offset = CSS_OUTLIER if is_css else float(np.linalg.norm(GYRO_OUTLIER))
        peak, recovery = peak_and_recovery(err, step, ambient)
        report.append((step, 'CSS ' if is_css else 'gyro', peak, peak / ambient, peak / offset, recovery))

    print("SunlineFilter outlier recovery")
    print("  ambient error: heading %.3e   rate %.3e   [cssSigma %.3e  gyroSigma %.3e]"
          % (heading_ambient, rate_ambient, cssSigma, gyroSigma))
    print("  step  chan  peak err    x ambient   absorbed   recovery")
    for step, chan, peak, xamb, absorbed, recovery in report:
        print("  %4d  %s  %.3e   %6.1fx    %6.2f%%   %s meas"
              % (step, chan, peak, xamb, 100 * absorbed, recovery))

    if show_plots:
        diff = np.copy(state_data_log)
        diff[:, 1:] -= expected[1:num_steps + 1, 1:]
        used = np.ones(num_steps, dtype=bool)
        heading_err_col = np.column_stack([state_data_log[:, 0], heading_err])
        rate_err_col = np.column_stack([state_data_log[:, 0], rate_err])
        filter_plots.outlier_rejection(css_meas, css_truth, used, 'CSS Outliers').show()
        filter_plots.outlier_rejection(gyro_meas, gyro_truth, used, 'Gyro Outliers').show()
        filter_plots.error_recovery(heading_err_col, CSS_OUTLIER_STEPS, heading_ambient, 'Heading').show()
        filter_plots.error_recovery(rate_err_col, GYRO_OUTLIER_STEPS, rate_ambient, 'Rate').show()
        filter_plots.state_covar(state_data_log, covariance_data_log, 'Outlier Recovery').show()
        filter_plots.states(diff, 'Outlier Recovery').show()

    for step, chan, peak, xamb, absorbed, recovery in report:
        ambient = heading_ambient if chan == 'CSS ' else rate_ambient
        assert peak <= peak_bound * ambient, \
            'the outlier at step %d moved the estimate to %g, beyond %gx the ambient error %g' \
            % (step, peak, peak_bound, ambient)
        assert recovery is not None and recovery <= recovery_bound, \
            'the estimate did not settle back within %d measurements of the outlier at step %d' \
            % (recovery_bound, step)

    # The clean measurements still do their job: the estimate tracks truth and the covariance shrinks.
    half_time = num_steps // 2
    np.testing.assert_allclose(state_data_log[half_time:, 1:4], expected[half_time + 1:num_steps + 1, 1:4],
                               atol=5 * cssSigma, err_msg='heading estimation error', verbose=True)
    np.testing.assert_allclose(state_data_log[half_time:, 4:7], expected[half_time + 1:num_steps + 1, 4:7],
                               atol=5 * gyroSigma, err_msg='rate estimation error', verbose=True)
    np.testing.assert_array_less(np.diag(covariance_data_log[-1, 1:num_states ** 2 + 1].reshape([num_states] * 2)),
                                 np.diag(covar_all[0, 1:num_states ** 2 + 1].reshape([num_states] * 2)),
                                 err_msg='covariance error', verbose=True)


if __name__ == "__main__":
    state_update_flyby(True, True)
    state_update_outlier_recovery(True)
