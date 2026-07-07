#
#   Unit Test Script
#   Module Name: sunTrackError
#
#   The combined algorithm input->output behavior is covered by the C++ integrated regression
#   test (test_sunTrackError_integrated.cpp). This pytest covers the adapter layer: SWIG property
#   round-trips, the message I/O boundary, and the optional-message-driven maneuver selection.
#

import numpy as np

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import sunTrackErrorF32
from xmera.utilities import macros
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.architecture import messaging

# Fixed, representative navigation/reference inputs shared by the adapter tests.
sigma_BN = [0.25, -0.45, 0.75]
omega_BN_B = [-0.015, -0.012, 0.005]
sigma_RN = [0.35, -0.25, 0.15]
omega_RN_N = [0.018, -0.032, 0.015]
domega_RN_N = [0.048, -0.022, 0.025]


def _run_sim(sun_avoidance):
    """Run the module through a short Xmera simulation. When sun_avoidance is True the optional
    translational-navigation and ephemeris messages are connected, which engages the maneuver."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess("TestProcess")
    testProc.addTask(unitTestSim.CreateNewTask("unitTask", testProcessRate))

    module = sunTrackErrorF32.SunTrackError()
    module.modelTag = "sunTrackError"
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

    if sun_avoidance:
        module.angleRate = 1 * np.pi / 180.0
        module.sensitiveHat_B = [0.0, -1.0, 0.0]
        transNavData = messaging.NavTransMsgF32Payload()
        transNavData.r_BN_N = [-30, 20, -50]
        transNavMsg = messaging.NavTransMsgF32().write(transNavData)
        module.transNavInMsg.subscribeTo(transNavMsg)
        ephemerisData = messaging.EphemerisMsgF32Payload()
        ephemerisData.r_BdyZero_N = np.array([1, 2, 3])
        ephemerisMsg = messaging.EphemerisMsgF32().write(ephemerisData)
        module.ephemerisInMsg.subscribeTo(ephemerisMsg)

    dataLog = module.attGuidOutMsg.recorder()
    unitTestSim.AddModelToTask("unitTask", dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(5.0))
    unitTestSim.ExecuteSimulation()

    return dataLog


def test_sunTrackError_config_roundtrip():
    """Public configuration properties round-trip through the SWIG interface."""
    module = sunTrackErrorF32.SunTrackError()
    module.modelTag = "sunTrackError"

    angleRate = 0.0123
    module.angleRate = angleRate
    np.testing.assert_allclose(module.angleRate, angleRate, rtol=1e-6, atol=1e-6)

    sensitiveHat_B = [0.1, -0.9, 0.2]
    module.sensitiveHat_B = sensitiveHat_B
    np.testing.assert_allclose(
        np.array(module.sensitiveHat_B).flatten(), sensitiveHat_B, rtol=1e-6, atol=1e-6
    )


def test_sunTrackError_no_maneuver_matches_tracking_error():
    """Without the optional messages, computeAngleStart is false and the module reduces to a plain
    attitude tracking error about the input reference frame. Verifies the adapter message I/O and
    the no-maneuver path end to end against an independent truth."""
    dataLog = _run_sim(sun_avoidance=False)

    BN = rbk.MRP2C(np.array(sigma_BN))
    RN = rbk.MRP2C(np.array(sigma_RN))
    BR = np.dot(BN, RN.T)

    tol = 1e-6
    np.testing.assert_allclose(dataLog.sigma_BR[-1], rbk.C2MRP(BR), rtol=tol, atol=tol)
    np.testing.assert_allclose(dataLog.omega_RN_B[-1], np.dot(BN, np.array(omega_RN_N)), rtol=tol, atol=tol)
    np.testing.assert_allclose(
        dataLog.omega_BR_B[-1], np.array(omega_BN_B) - np.dot(BN, np.array(omega_RN_N)), rtol=tol, atol=tol
    )
    np.testing.assert_allclose(dataLog.domega_RN_B[-1], np.dot(BN, np.array(domega_RN_N)), rtol=tol, atol=tol)


def test_sunTrackError_optional_messages_engage_maneuver():
    """Subscribing the optional trans/ephemeris messages engages the Sun-avoidance maneuver
    (computeAngleStart is derived from the link state), shifting the guidance output away from the
    no-maneuver tracking error. Exercises the optional-message wiring and the double[3]->float
    position/ephemeris conversion path."""
    off = _run_sim(sun_avoidance=False)
    on = _run_sim(sun_avoidance=True)

    assert np.all(np.isfinite(on.sigma_BR))
    assert np.all(np.isfinite(on.omega_BR_B))
    assert np.all(np.isfinite(on.omega_RN_B))
    assert np.all(np.isfinite(on.domega_RN_B))

    # The engaged maneuver changes the attitude error and the reference rate relative to
    # the maneuver-off tracking error.
    assert not np.allclose(on.sigma_BR, off.sigma_BR, atol=1e-4)
    assert not np.allclose(on.omega_RN_B, off.omega_RN_B, atol=1e-4)


if __name__ == "__main__":
    test_sunTrackError_config_roundtrip()
    test_sunTrackError_no_maneuver_matches_tracking_error()
    test_sunTrackError_optional_messages_engage_maneuver()
