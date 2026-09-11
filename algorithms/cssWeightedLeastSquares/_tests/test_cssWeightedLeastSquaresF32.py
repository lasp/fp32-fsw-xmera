import numpy as np
import pytest

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import cssWeightedLeastSquaresF32
from xmera.utilities import macros
from xmera.architecture import messaging

# Eight-sensor coarse sun sensor constellation: two opposing four-sensor pyramids, so a sun heading
# along any body axis lights at least three sensors and the least squares fit is over-determined.
CSS_ORIENTATIONS = [
    [0.70710678118654746, -0.5, 0.5],
    [0.70710678118654746, -0.5, -0.5],
    [0.70710678118654746, 0.5, -0.5],
    [0.70710678118654746, 0.5, 0.5],
    [-0.70710678118654746, 0.0, 0.70710678118654757],
    [-0.70710678118654746, 0.70710678118654757, 0.0],
    [-0.70710678118654746, 0.0, -0.70710678118654757],
    [-0.70710678118654746, -0.70710678118654757, 0.0],
]

# [-] cosine at or below which a reading is treated as noise and dropped from the fit
SENSOR_USE_THRESH = 0.15

PRINCIPAL_AXES = [
    [1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 0.0, -1.0],
]

# Headings for the decreasing-coverage test: the first lights five sensors and the second three, so both
# stay in the weighted least squares branch while the active count drops between cycles.
MANY_ACTIVE_HEADING = [0.3342, -0.6230, 0.7073]
FEW_ACTIVE_HEADING = [0.3635, 0.8643, 0.3476]

# A heading 40.68 degrees off the +z axis in the x-z plane, which lights only sensors 0 and 3.
LOW_COVERAGE_LATITUDE = np.deg2rad(40.68)
LOW_COVERAGE_HEADING = [np.sin(LOW_COVERAGE_LATITUDE), 0.0, np.cos(LOW_COVERAGE_LATITUDE)]


def cos_values(sun_heading_B):
    """Per-sensor cosine readings for a sun heading. A coarse sun sensor cannot report a negative
    cosine, so a sensor facing away from the sun reads zero rather than the signed dot product."""
    return [max(float(np.dot(sun_heading_B, n_hat_B)), 0.0) for n_hat_B in CSS_ORIENTATIONS]



def css_config_msg():
    """The constellation geometry message. The module reads the sensor layout from here rather than
    from properties, so one message can configure every estimator that shares the array."""
    css_config_data = messaging.CSSConfigMsgF32Payload()
    sensors = []
    for n_hat_B in CSS_ORIENTATIONS:
        sensor = messaging.CSSUnitConfigMsgF32Payload()
        sensor.nHat_B = n_hat_B
        sensor.CBias = 1.0
        sensors.append(sensor)
    css_config_data.nCSS = len(CSS_ORIENTATIONS)
    css_config_data.cssVals = sensors
    return messaging.CSSConfigMsgF32().write(css_config_data)


def weighted_fit(cos_readings):
    """The weighted least squares heading over the lit sensors, solved in double precision."""
    lit = [index for index, reading in enumerate(cos_readings) if reading > SENSOR_USE_THRESH]
    observations = np.array([CSS_ORIENTATIONS[index] for index in lit], dtype=float)
    measurements = np.array([cos_readings[index] for index in lit], dtype=float)
    weights = np.diag(measurements)
    heading = np.linalg.solve(observations.T @ weights @ observations, observations.T @ weights @ measurements)
    return heading / np.linalg.norm(heading)


@pytest.mark.parametrize("sun_heading_B", PRINCIPAL_AXES)
def test_css_weighted_least_squares_nominal(sun_heading_B):
    """Nominal Unit Test: full coverage along each body axis"""
    cos_readings = cos_values(sun_heading_B)

    # The constellation is symmetric about every body axis, so the normal matrix comes out diagonal and
    # the fit returns the true heading exactly. Every lit sensor is then predicted exactly and every
    # sensor facing away is predicted at the zero it reported, so the whole residual vector is zero.
    expected_residuals = np.zeros(len(CSS_ORIENTATIONS))

    # That symmetry also makes the weights drop out of the normal equations, so the weighted and
    # unweighted fits agree and both paths can be checked against the same truth.
    run_test(cos_readings, sun_heading_B, expected_residuals=expected_residuals)
    run_test(cos_readings, sun_heading_B, expected_residuals=expected_residuals, use_weights=True)


def test_css_weighted_least_squares_two_sensor_coverage():
    """Off Nominal Unit Test: two lit sensors, an exactly determined minimum norm fit"""
    cos_readings = cos_values(LOW_COVERAGE_HEADING)

    # Sensors 0 and 3 are lit and read the same cosine, so the minimum norm solution is the direction
    # that bisects their two boresights. It is 14 degrees off the true heading, which is the price of
    # the missing third measurement rather than an error in the fit.
    bisector = np.array(CSS_ORIENTATIONS[0]) + np.array(CSS_ORIENTATIONS[3])
    expected_heading = bisector / np.linalg.norm(bisector)

    run_test(cos_readings, expected_heading)


def test_css_weighted_least_squares_single_sensor_coverage():
    """Off Nominal Unit Test: one lit sensor, so the fit can only report a guess"""
    cos_readings = cos_values(LOW_COVERAGE_HEADING)
    cos_readings[0] = 0.0  # blind sensor 0, leaving sensor 3 as the only reading above threshold

    # One reading fixes only the cone of headings about that sensor's boresight, so the estimator
    # returns the boresight itself. That is a guess on the cone, not an estimate of the heading.
    run_test(cos_readings, CSS_ORIENTATIONS[3])


def test_css_weighted_least_squares_no_signal():
    """Off Nominal Unit Test: no reading above threshold, so there is no sun to estimate"""
    cos_readings = [0.0] * len(CSS_ORIENTATIONS)

    # With no sun the module reports the zero vector rather than a stale or invented heading, and no
    # sensor contributed an observation, so no residual is reported either.
    run_test(cos_readings, np.zeros(3), expected_residuals=np.zeros(len(CSS_ORIENTATIONS)))


def test_css_weighted_least_squares_rate_estimate():
    """Module Unit Test: the rate from two successive headings, and its reset behavior"""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = cssWeightedLeastSquaresF32.CssWeightedLeastSquares()
    module.modelTag = "cssWeightedLeastSquares"

    config_in_msg = css_config_msg()
    module.cssConfigInMsg.subscribeTo(config_in_msg)
    module.useWeights = False
    module.sensorUseThresh = SENSOR_USE_THRESH
    module.controlPeriod = macros.NANO2SEC * test_process_rate

    unit_test_sim.AddModelToTask(unit_task_name, module)

    input_message_data = messaging.CSSArraySensorMsgF32Payload()
    input_message_data.CosValue = cos_values([1.0, 0.0, 0.0])
    in_msg = messaging.CSSArraySensorMsgF32().write(input_message_data)
    module.cssDataInMsg.subscribeTo(in_msg)

    data_log = module.navStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    # Slew the sun 90 degrees about +z within one 0.5 s step, an apparent rate of pi rad/s about -z.
    input_message_data.CosValue = cos_values([0.0, 1.0, 0.0])
    in_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))
    unit_test_sim.ExecuteSimulation()

    # A reset discards the prior heading, so the step across it must not be differenced into a rate.
    module.reset(1)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.5))
    unit_test_sim.ExecuteSimulation()

    input_message_data.CosValue = cos_values([1.0, 0.0, 0.0])
    in_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(3.0))
    unit_test_sim.ExecuteSimulation()

    slew_rate = np.pi  # [r/s] 90 degrees swept in the 0.5 s task period
    expected_angular_velocity = [
        [0.0, 0.0, 0.0],  # t=0.0, first heading, nothing to difference against
        [0.0, 0.0, 0.0],  # t=0.5, heading unchanged
        [0.0, 0.0, 0.0],  # t=1.0, heading unchanged
        [0.0, 0.0, -slew_rate],  # t=1.5, sun stepped from +x to +y
        [0.0, 0.0, 0.0],  # t=2.0, heading unchanged
        [0.0, 0.0, 0.0],  # t=2.5, first heading after the reset
        [0.0, 0.0, slew_rate],  # t=3.0, sun stepped back from +y to +x
    ]

    # The rate is an arc cosine divided by the task period, so float32 round-off on the heading is
    # amplified by 1/dt; 1e-5 sits an order of magnitude above the observed residual.
    np.testing.assert_allclose(data_log.omega_BN_B, expected_angular_velocity, rtol=1e-5, atol=1e-5, verbose=True)


def test_css_weighted_least_squares_reinitialize():
    """Module Unit Test: reInitialize() drops the prior heading at a state transition"""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = cssWeightedLeastSquaresF32.CssWeightedLeastSquares()
    module.modelTag = "cssWeightedLeastSquares"

    config_in_msg = css_config_msg()
    module.cssConfigInMsg.subscribeTo(config_in_msg)
    module.useWeights = False
    module.sensorUseThresh = SENSOR_USE_THRESH
    module.controlPeriod = macros.NANO2SEC * test_process_rate

    unit_test_sim.AddModelToTask(unit_task_name, module)

    input_message_data = messaging.CSSArraySensorMsgF32Payload()
    input_message_data.CosValue = cos_values([1.0, 0.0, 0.0])
    in_msg = messaging.CSSArraySensorMsgF32().write(input_message_data)
    module.cssDataInMsg.subscribeTo(in_msg)

    data_log = module.navStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(test_process_rate)
    unit_test_sim.ExecuteSimulation()

    module.reInitialize()

    # The same 90 degree step that gives pi rad/s above now spans the re-initialization, so the prior
    # heading is gone and the step is discarded rather than reported as a rate.
    input_message_data.CosValue = cos_values([0.0, 1.0, 0.0])
    in_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    np.testing.assert_allclose(data_log.omega_BN_B[-1], np.zeros(3), rtol=0, atol=0, verbose=True)


def test_css_weighted_least_squares_reconfigure():
    """Module Unit Test: reconfigure() pushes an edited threshold onto the live algorithm"""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = cssWeightedLeastSquaresF32.CssWeightedLeastSquares()
    module.modelTag = "cssWeightedLeastSquares"

    config_in_msg = css_config_msg()
    module.cssConfigInMsg.subscribeTo(config_in_msg)
    module.useWeights = False
    module.sensorUseThresh = SENSOR_USE_THRESH
    module.controlPeriod = macros.NANO2SEC * test_process_rate

    unit_test_sim.AddModelToTask(unit_task_name, module)

    # This heading reads 0.5 on two sensors and 0.7071 on a third, so raising the threshold past 0.5
    # drops the fit from three sensors to one.
    input_message_data = messaging.CSSArraySensorMsgF32Payload()
    input_message_data.CosValue = cos_values([0.0, 1.0, 0.0])
    in_msg = messaging.CSSArraySensorMsgF32().write(input_message_data)
    module.cssDataInMsg.subscribeTo(in_msg)

    data_log = module.logger("numActiveCss")
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(test_process_rate)
    unit_test_sim.ExecuteSimulation()

    np.testing.assert_array_equal(data_log.numActiveCss[-1], 3)

    module.sensorUseThresh = 0.6
    module.reconfigure()

    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    np.testing.assert_array_equal(data_log.numActiveCss[-1], 1)
    np.testing.assert_allclose(module.sensorUseThresh, 0.6, rtol=0, atol=1e-7, verbose=True)


def test_css_weighted_least_squares_decreasing_coverage():
    """Module Unit Test: a cycle whose lit sensor count is lower than the cycle before it"""
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = cssWeightedLeastSquaresF32.CssWeightedLeastSquares()
    module.modelTag = "cssWeightedLeastSquares"

    config_in_msg = css_config_msg()
    module.cssConfigInMsg.subscribeTo(config_in_msg)
    module.useWeights = True
    module.sensorUseThresh = SENSOR_USE_THRESH
    module.controlPeriod = macros.NANO2SEC * test_process_rate

    unit_test_sim.AddModelToTask(unit_task_name, module)

    many_readings = cos_values(MANY_ACTIVE_HEADING)
    few_readings = cos_values(FEW_ACTIVE_HEADING)

    input_message_data = messaging.CSSArraySensorMsgF32Payload()
    input_message_data.CosValue = many_readings
    in_msg = messaging.CSSArraySensorMsgF32().write(input_message_data)
    module.cssDataInMsg.subscribeTo(in_msg)

    data_log = module.navStateOutMsg.recorder()
    num_active_log = module.logger("numActiveCss")
    unit_test_sim.AddModelToTask(unit_task_name, data_log)
    unit_test_sim.AddModelToTask(unit_task_name, num_active_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(test_process_rate)
    unit_test_sim.ExecuteSimulation()

    input_message_data.CosValue = few_readings
    in_msg.write(input_message_data)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.5))
    unit_test_sim.ExecuteSimulation()

    np.testing.assert_equal(num_active_log.numActiveCss[1], 5)
    np.testing.assert_equal(num_active_log.numActiveCss[-1], 3)

    # The fit is assembled in buffers sized for the full sensor complement, whose entries past the active
    # count are zero. A row left behind by the five-sensor cycle would bias the three-sensor fit, so both
    # cycles are checked against the double-precision solution over their own lit sensors. The tolerance is
    # float32 round-off on a 3x3 solve over unit-norm data.
    np.testing.assert_allclose(
        data_log.vehSunPntBdy[1], weighted_fit(many_readings), atol=1e-6, rtol=1e-6, verbose=True
    )
    np.testing.assert_allclose(
        data_log.vehSunPntBdy[-1], weighted_fit(few_readings), atol=1e-6, rtol=1e-6, verbose=True
    )


def run_test(
    cos_readings,
    expected_heading,
    expected_residuals=None,
    use_weights=False,
):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    sim_time = 0.5
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = cssWeightedLeastSquaresF32.CssWeightedLeastSquares()
    module.modelTag = "cssWeightedLeastSquares"

    config_in_msg = css_config_msg()
    module.cssConfigInMsg.subscribeTo(config_in_msg)
    module.useWeights = use_weights
    module.sensorUseThresh = SENSOR_USE_THRESH
    module.controlPeriod = macros.NANO2SEC * test_process_rate

    unit_test_sim.AddModelToTask(unit_task_name, module)

    input_message_data = messaging.CSSArraySensorMsgF32Payload()
    input_message_data.CosValue = cos_readings
    in_msg = messaging.CSSArraySensorMsgF32().write(input_message_data)
    module.cssDataInMsg.subscribeTo(in_msg)

    nav_data_log = module.navStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, nav_data_log)
    filter_data_log = module.filterCssResOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)
    num_active_data_log = module.logger("numActiveCss")
    unit_test_sim.AddModelToTask(unit_task_name, num_active_data_log)

    unit_test_sim.InitializeSimulation()

    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))

    unit_test_sim.ExecuteSimulation()

    module_output_heading = nav_data_log.vehSunPntBdy
    module_output_residuals = filter_data_log.postFits
    module_output_num_active = num_active_data_log.numActiveCss

    # The estimator drops every reading at or below the threshold, so this is the count it must report.
    expected_num_active = sum(1 for reading in cos_readings if reading > SENSOR_USE_THRESH)

    np.testing.assert_allclose(module_output_heading[-1], expected_heading, rtol=1e-6, atol=1e-6, verbose=True)
    np.testing.assert_array_equal(module_output_num_active[-1], expected_num_active)
    np.testing.assert_array_equal(filter_data_log.sizeOfObservations[-1], expected_num_active)
    if expected_residuals is not None:
        np.testing.assert_allclose(
            module_output_residuals[-1][: len(CSS_ORIENTATIONS)], expected_residuals, rtol=0, atol=1e-6, verbose=True
        )
    np.testing.assert_array_equal(module.useWeights, use_weights)
    np.testing.assert_allclose(module.sensorUseThresh, SENSOR_USE_THRESH, rtol=0, atol=1e-7, verbose=True)


if __name__ == "__main__":
    test_css_weighted_least_squares_two_sensor_coverage()
