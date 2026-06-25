import inspect
import math
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
import spiceypy
from xmera.architecture import messaging
from xmera.fp32 import oeStateEphemF32
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import orbitalMotion

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('fswAlgorithms')
from xmera import __path__
bskPath = __path__[0]

relative_error = 0.001  # 0.1% relative error maximum
colors = ['r','g','b']

@pytest.mark.parametrize('valid_curve, anomaly_flag', [
    (True, oeStateEphemF32.TRUE_ANOMALY),
    (True, oeStateEphemF32.MEAN_ANOMALY),
    (False, oeStateEphemF32.TRUE_ANOMALY)
])
def test_cheby_fit(show_plots, valid_curve, anomaly_flag):
    """Module Unit Test"""
    cheby_fit(show_plots, valid_curve, anomaly_flag)

def test_zero_inputs(show_plots):
    """Module Unit Test"""
    task_name = "unitTask"  # arbitrary name (don't change)
    process_name = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()

    test_process = sim.CreateNewProcess(process_name)
    # create the dynamics task and specify the integration update time
    test_process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(1)))

    oe_ephemeris_module = oeStateEphemF32.OEStateEphem()
    oe_ephemeris_module.modelTag = "oe_ephemeris_module"
    sim.AddModelToTask(task_name, oe_ephemeris_module)

    oe_ephemeris_module.centralBodyGravitationalParameter = 0

    oe_ephemeris_module.setArcRadiusPeriapsisCoefficients(0, [0] * 20)
    oe_ephemeris_module.setArcEccentricityCoefficients(0, [0] * 20)
    oe_ephemeris_module.setArcInclinationCoefficients(0, [0] * 20)
    oe_ephemeris_module.setArcArgPeriapsisCoefficients(0, [0] * 20)
    oe_ephemeris_module.setArcTrueAnomalyCoefficients(0, [0] * 20)
    oe_ephemeris_module.setArcRaanCoefficients(0, [0] * 20)
    oe_ephemeris_module.setArcNumberOfCoefficients(0, 1)
    oe_ephemeris_module.setArcMiddleTime(0, 1)
    oe_ephemeris_module.setArcRadiusTime(0, 1/2.0)
    oe_ephemeris_module.setArcAnomalyFlag(0, oeStateEphemF32.TRUE_ANOMALY)

    clock_correlation_data = messaging.TDBVehicleClockCorrelationMsgPayload()
    clock_correlation_data.vehicleClockTime = 0.0
    clock_correlation_data.ephemerisTime = oe_ephemeris_module.getArcMiddleTime(0) - oe_ephemeris_module.getArcRadiusTime(0)

    clock_input_msg = messaging.TDBVehicleClockCorrelationMsg().write(clock_correlation_data)
    oe_ephemeris_module.clockCorrInMsg.subscribeTo(clock_input_msg)

    ephemeris_log = oe_ephemeris_module.stateFitOutMsg.recorder()
    sim.AddModelToTask(task_name, ephemeris_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(int(1*1.0E9))
    sim.ExecuteSimulation()

    ephemeris_positions = ephemeris_log.r_BdyZero_N
    ephemeris_velocities = ephemeris_log.v_BdyZero_N

    accuracy = 1e-7

    np.testing.assert_allclose(ephemeris_positions, 0, atol=accuracy, rtol=0, err_msg="position values should have been zero")
    np.testing.assert_allclose(ephemeris_velocities, 0, atol=accuracy, rtol=0, err_msg="velocity values should have been zero")

def cheby_fit(show_plots, valid_curve, anomaly_flag):
    number_curve_points = 4*8640+1
    curve_duration_sec = 4*86400
    log_rate = curve_duration_sec // (number_curve_points - 1)
    number_cheby_coefficients = 14
    frame = "j2000"
    zero_base = "Earth"
    central_body_mu = 3.98574405096E14

    spice_time = "2015 April 10, 00:00:00.0 TDB"
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/naif0012.tls')
    et = spiceypy.str2et(spice_time)
    start_time_et = et
    end_time_et = start_time_et + curve_duration_sec

    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/de430.bsp')
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/naif0012.tls')
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    spiceypy.furnsh(path + '/TDRSS.bsp')

    true_positions_m = []
    true_velocities_mps = []
    time_array = np.linspace(start_time_et, end_time_et, number_curve_points)
    radius_periapsis = []
    eccentricity = []
    inclination = []
    raan = []
    omega = []
    true_anomaly = []
    previous_anomaly = 0.0
    anomaly_switch_count = 0

    for timeVal in time_array:
        current_time = spiceypy.et2utc(timeVal, 'C', 4, 1024)
        [spice_state, _] = spiceypy.spkezr('-221', spiceypy.str2et(current_time), frame, 'NONE', zero_base)
        position = spice_state[0:3]
        velocity = spice_state[3:6]
        orbital_elements = orbitalMotion.rv2elem(central_body_mu*1e-9, position, velocity)
        true_positions_m.append(position*1e3)
        true_velocities_mps.append(velocity*1e3)
        radius_periapsis.append(orbital_elements.rPeriap * 1000.0)
        eccentricity.append(orbital_elements.e)
        inclination.append(orbital_elements.i)
        raan.append(orbital_elements.Omega)
        omega.append(orbital_elements.omega)
        if anomaly_flag == oeStateEphemF32.MEAN_ANOMALY:
            current_anomaly = orbitalMotion.E2M(orbitalMotion.f2E(orbital_elements.f, orbital_elements.e), orbital_elements.e)
        else:
            current_anomaly = orbital_elements.f
        if current_anomaly < previous_anomaly:
            anomaly_switch_count += 1
        true_anomaly.append(2*math.pi*anomaly_switch_count + current_anomaly)
        previous_anomaly = current_anomaly

    true_positions_m = np.array(true_positions_m)
    true_velocities_mps = np.array(true_velocities_mps)

    fit_times = np.linspace(-1, 1, number_curve_points)
    radius_periasis_fit = np.polynomial.chebyshev.chebfit(fit_times, radius_periapsis, number_cheby_coefficients - 1) # np chebfit takes in the degree, not the number of coefficients
    eccentricity_fit = np.polynomial.chebyshev.chebfit(fit_times, eccentricity, number_cheby_coefficients - 1)
    inclination_fit = np.polynomial.chebyshev.chebfit(fit_times, inclination, number_cheby_coefficients - 1)
    rann_fit = np.polynomial.chebyshev.chebfit(fit_times, raan, number_cheby_coefficients - 1)
    omega_fit = np.polynomial.chebyshev.chebfit(fit_times, omega, number_cheby_coefficients - 1)
    anomaly_fit = np.polynomial.chebyshev.chebfit(fit_times, true_anomaly, number_cheby_coefficients - 1)

    task_name = "unitTask"  # arbitrary name (don't change)
    process_name = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()

    test_process = sim.CreateNewProcess(process_name)
    # create the dynamics task and specify the integration update time
    test_process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(log_rate)))

    oe_ephemeris_module = oeStateEphemF32.OEStateEphem()
    oe_ephemeris_module.modelTag = "oe_ephemeris_module"
    sim.AddModelToTask(task_name, oe_ephemeris_module)

    oe_ephemeris_module.centralBodyGravitationalParameter = central_body_mu

    oe_ephemeris_module.setArcRadiusPeriapsisCoefficients(0, radius_periasis_fit.tolist() + [0] * (20 - number_cheby_coefficients))
    oe_ephemeris_module.setArcEccentricityCoefficients(0, eccentricity_fit.tolist() + [0] * (20 - number_cheby_coefficients))
    oe_ephemeris_module.setArcInclinationCoefficients(0, inclination_fit.tolist() + [0] * (20 - number_cheby_coefficients))
    oe_ephemeris_module.setArcArgPeriapsisCoefficients(0, omega_fit.tolist() + [0] * (20 - number_cheby_coefficients))
    oe_ephemeris_module.setArcTrueAnomalyCoefficients(0, anomaly_fit.tolist() + [0] * (20 - number_cheby_coefficients))
    oe_ephemeris_module.setArcRaanCoefficients(0, rann_fit.tolist() + [0] * (20 - number_cheby_coefficients))
    oe_ephemeris_module.setArcNumberOfCoefficients(0, number_cheby_coefficients)
    oe_ephemeris_module.setArcMiddleTime(0, start_time_et + curve_duration_sec/2.0)
    oe_ephemeris_module.setArcRadiusTime(0, curve_duration_sec/2.0)

    if anomaly_flag is not None:
        oe_ephemeris_module.setArcAnomalyFlag(0, anomaly_flag)

    clock_correlation_data = messaging.TDBVehicleClockCorrelationMsgPayload()
    clock_correlation_data.vehicleClockTime = 0.0
    clock_correlation_data.ephemerisTime = oe_ephemeris_module.getArcMiddleTime(0) - oe_ephemeris_module.getArcRadiusTime(0)

    clock_input_msg = messaging.TDBVehicleClockCorrelationMsg().write(clock_correlation_data)
    oe_ephemeris_module.clockCorrInMsg.subscribeTo(clock_input_msg)

    ephemeris_log = oe_ephemeris_module.stateFitOutMsg.recorder()
    sim.AddModelToTask(task_name, ephemeris_log)

    if not valid_curve:
        sim.InitializeSimulation()
        sim.ConfigureStopTime(int((curve_duration_sec + log_rate) * 1.0E9))
        sim.ExecuteSimulation()
    else:
        sim.InitializeSimulation()
        sim.ConfigureStopTime(int(curve_duration_sec*1.0E9))
        sim.ExecuteSimulation()

    ephemeris_positions = ephemeris_log.r_BdyZero_N
    ephemeris_velocities = ephemeris_log.v_BdyZero_N

    if not valid_curve:
        last_log_index = (curve_duration_sec + log_rate) // log_rate - 1
        second_last_position = ephemeris_positions[last_log_index + 1, 0:] - true_positions_m[last_log_index, :]
        last_position = ephemeris_positions[last_log_index, 0:] - true_positions_m[last_log_index, :]

        np.testing.assert_array_equal(second_last_position, last_position, "Expected Chebychev position to rail high or low")

        second_last_velocity = ephemeris_velocities[last_log_index + 1, 0:] - true_velocities_mps[last_log_index, :]
        last_velocity = ephemeris_velocities[last_log_index, 0:] - true_velocities_mps[last_log_index, :]
        np.testing.assert_array_equal(second_last_velocity, last_velocity, "Expected Chebychev velocity to rail high or low")

    else:
        pox_vector_err = []
        vel_vector_err = []
        for i in range(len(ephemeris_positions[:, 0])):
            pox_vector_err.append(np.linalg.norm(ephemeris_positions[i, :] - true_positions_m[i, :])/np.linalg.norm(true_positions_m[i, :]))
            vel_vector_err.append(np.linalg.norm(ephemeris_velocities[i, :] - true_velocities_mps[i, :])/np.linalg.norm(true_velocities_mps[i, :]))

        np.testing.assert_array_less(max(pox_vector_err), relative_error, "mas_pox_vector_err >= relative_error")
        np.testing.assert_array_less(max(vel_vector_err), relative_error, "max_vel_vector_err >= relative_error")

        plt.close("all")
        # plot the fitted and actual position coordinates
        plt.figure(1)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        for idx in range(0, 3):
            plt.plot(ephemeris_log.times()*macros.NANO2HOUR,
                     ephemeris_positions[:, idx]/1000,
                     color=colors[idx],
                     linewidth=0.5,
                     label='$r_{fit,' + str(idx) + '}$')
            plt.plot(ephemeris_log.times()*macros.NANO2HOUR,
                     true_positions_m[:, idx]/1000,
                     color=colors[idx],
                     linestyle='dashed', linewidth=2,
                     label='$r_{true,' + str(idx) + '}$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Inertial Position [km]')

        # plot the fitted and actual velocity coordinates
        plt.figure(2)
        for idx in range(0, 3):
            plt.plot(ephemeris_log.times()*macros.NANO2HOUR,
                     ephemeris_velocities[:, idx]/1000,
                     color=colors[idx],
                     linewidth=0.5,
                     label='$v_{fit,' + str(idx) + '}$')
            plt.plot(ephemeris_log.times()*macros.NANO2HOUR,
                     true_velocities_mps[:, idx]/1000,
                     color=colors[idx],
                     linestyle='dashed', linewidth=2,
                     label='$v_{true,' + str(idx) + '}$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Velocity [km/s]')

        # plot the difference in position coordinates
        plt.figure(3)
        array_length = ephemeris_positions[:, 0].size
        for idx in range(0,3):
            plt.plot(ephemeris_log.times() * macros.NANO2HOUR,
                     ephemeris_positions[:, idx] - true_positions_m[:, idx],
                     color=colors[idx],
                     linewidth=0.5,
                     label=r'$\Delta r_{' + str(idx) + '}$')
        plt.plot(ephemeris_log.times() * macros.NANO2HOUR,
                 relative_error*np.ones(array_length),
                 color='r',
                 linewidth=1)
        plt.plot(ephemeris_log.times() * macros.NANO2HOUR,
                 -relative_error * np.ones(array_length),
                 color='r',
                 linewidth=1)
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Position Difference [m]')

        # plot the difference in velocity coordinates
        plt.figure(4)
        array_length = ephemeris_velocities[:, 0].size
        for idx in range(0,3):
            plt.plot(ephemeris_log.times() * macros.NANO2HOUR,
                     ephemeris_velocities[:, idx] - true_velocities_mps[:, idx],
                     color=colors[idx],
                     linewidth=0.5,
                     label=r'$\Delta v_{' + str(idx) + '}$')
        plt.plot(ephemeris_log.times() * macros.NANO2HOUR,
                 relative_error*np.ones(array_length),
                 color='r',
                 linewidth=1)
        plt.plot(ephemeris_log.times() * macros.NANO2HOUR,
                 -relative_error * np.ones(array_length),
                 color='r',
                 linewidth=1)
        plt.legend(loc='lower right')
        plt.xlabel('Time [h]')
        plt.ylabel('Velocity Difference [m/s]')

    if show_plots:
        plt.show()
        plt.close('all')



if __name__ == "__main__":
    cheby_fit(True,        # showPlots
                       True,        # valid_curve
                       oeStateEphemF32.MEAN_ANOMALY)  # anomaly_flag
