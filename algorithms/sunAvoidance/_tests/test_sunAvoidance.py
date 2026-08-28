#
#   Unit Test Script
#   Module Name: sunAvoidance
#
#   sunAvoidance produces the Sun-avoidance maneuver-adjusted reference frame; attTrackingError is
#   chained downstream to form the attitude tracking error. The combined algorithm behavior is covered
#   by the C++ integrated regression test (test_sunAvoidance_integrated.cpp). This pytest covers the
#   adapter layer: SWIG property round-trips and the message I/O boundary.
#

import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import sunAvoidanceF32
from xmera.utilities import macros
from xmera.architecture import messaging

# Fixed, representative navigation/reference inputs shared by the adapter tests.
sigma_BN = [0.25, -0.45, 0.75]
omega_BN_B = [-0.015, -0.012, 0.005]
sigma_RN = [0.35, -0.25, 0.15]
omega_RN_N = [0.018, -0.032, 0.015]
domega_RN_N = [0.048, -0.022, 0.025]


def _run_sim(sun_visible):
    """Run the module through a short Xmera simulation. All four input messages are connected: Sun
    avoidance is not optional, so the translational-navigation and ephemeris messages are required.
    When sun_visible is False the ephemeris carries a zero Sun position, which leaves no usable Sun
    direction and the module passes the input reference through."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess("TestProcess")
    testProc.addTask(unitTestSim.CreateNewTask("unitTask", testProcessRate))

    module = sunAvoidanceF32.SunAvoidance()
    module.modelTag = "sunAvoidance"
    unitTestSim.AddModelToTask("unitTask", module)

    NavStateOutData = messaging.NavAttMsgF32Payload()
    NavStateOutData.sigma_BN = sigma_BN
    NavStateOutData.omega_BN_B = omega_BN_B
    navStateInMsg = messaging.NavAttMsgF32().write(NavStateOutData)

    RefStateOutData = messaging.AttRefMsgF32Payload()
    RefStateOutData.sigma_RN = sigma_RN
    RefStateOutData.omega_RN_N = omega_RN_N
    RefStateOutData.domega_RN_N = domega_RN_N
    refInMsg = messaging.AttRefMsgF32().write(RefStateOutData)

    module.attNavInMsg.subscribeTo(navStateInMsg)
    module.attRefInMsg.subscribeTo(refInMsg)

    # slewRate must be a valid (positive) rate whether or not there is a maneuver to feed forward.
    module.slewRate = 1 * np.pi / 180.0
    module.sensitiveHat_B = [0.0, -1.0, 0.0]

    transNavData = messaging.NavTransMsgF32Payload()
    transNavData.r_BN_N = [-30, 20, -50]
    transNavMsg = messaging.NavTransMsgF32().write(transNavData)
    module.transNavInMsg.subscribeTo(transNavMsg)

    ephemerisData = messaging.EphemerisMsgF32Payload()
    ephemerisData.r_BdyZero_N = np.array([1, 2, 3]) if sun_visible else np.zeros(3)
    ephemerisMsg = messaging.EphemerisMsgF32().write(ephemerisData)
    module.ephemerisInMsg.subscribeTo(ephemerisMsg)

    dataLog = module.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask("unitTask", dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(5.0))
    unitTestSim.ExecuteSimulation()

    return dataLog


def test_sunAvoidance_config_roundtrip():
    """Public configuration properties round-trip through the SWIG interface."""
    module = sunAvoidanceF32.SunAvoidance()
    module.modelTag = "sunAvoidance"

    slewRate = 0.0123
    module.slewRate = slewRate
    np.testing.assert_allclose(module.slewRate, slewRate, rtol=1e-6, atol=1e-6)

    sensitiveHat_B = [0.1, -0.9, 0.2]
    module.sensitiveHat_B = sensitiveHat_B
    np.testing.assert_allclose(
        np.array(module.sensitiveHat_B).flatten(), sensitiveHat_B, rtol=1e-6, atol=1e-6
    )


def test_sunAvoidance_no_sun_information_passes_reference_through():
    """A zero Sun position leaves no usable Sun direction, so no maneuver is applied and the output
    reference frame equals the input reference. Verifies the adapter message I/O end to end."""
    dataLog = _run_sim(sun_visible=False)

    tol = 1e-6
    np.testing.assert_allclose(dataLog.sigma_RN[-1], sigma_RN, rtol=tol, atol=tol)
    np.testing.assert_allclose(dataLog.omega_RN_N[-1], omega_RN_N, rtol=tol, atol=tol)
    np.testing.assert_allclose(dataLog.domega_RN_N[-1], domega_RN_N, rtol=tol, atol=tol)


def test_sunAvoidance_engages_maneuver():
    """A usable Sun direction engages the Sun-avoidance maneuver, which rotates the output reference
    frame away from the input reference. Exercises the required-message wiring and the double[3]->float
    position/ephemeris conversion path."""
    on = _run_sim(sun_visible=True)

    assert np.all(np.isfinite(on.sigma_RN))
    assert np.all(np.isfinite(on.omega_RN_N))
    assert np.all(np.isfinite(on.domega_RN_N))

    # The engaged maneuver rotates the reference attitude and adds a feed-forward rate.
    assert not np.allclose(on.sigma_RN, sigma_RN, atol=1e-4)
    assert not np.allclose(on.omega_RN_N, omega_RN_N, atol=1e-4)
    # The constant-rate maneuver adds no angular acceleration.
    np.testing.assert_allclose(on.domega_RN_N[-1], domega_RN_N, rtol=1e-6, atol=1e-6)


if __name__ == "__main__":
    test_sunAvoidance_config_roundtrip()
    test_sunAvoidance_no_sun_information_passes_reference_through()
    test_sunAvoidance_engages_maneuver()
