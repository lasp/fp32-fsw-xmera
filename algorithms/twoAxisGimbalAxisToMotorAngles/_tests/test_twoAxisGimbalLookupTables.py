import inspect
import numpy as np
import os
import pytest
import sys
from xmera.ema import twoAxisGimbal
from xmera.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + "/../../../../..")

from supportData.sep_gimbal_interpolation.sep_gimbal_interpolation import gimbal_to_motor_angles
from supportData.sep_gimbal_interpolation.sep_gimbal_interpolation import load_interpolation_tables


def test_twoAxisGimbalLookupTables_gimbalToMotorAngles_noInterpolation():
    # Load the gimbal interpolation tables
    (
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    ) = load_interpolation_tables()

    # Create the gimbal lookup table class
    gimbalLookupTables = twoAxisGimbal.TwoAxisGimbalLookupTables(
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    )

    # Set the gimbal reference angles
    gimbalTipAngleRef = 0.0
    gimbalTiltAngleRef = 0.0

    # Interpolate the motor angles using the gimbal lookup table module
    interpolatedAngles = gimbalLookupTables.gimbalAnglesToMotorAngles(
        gimbalTipAngleRef * macros.D2R, gimbalTiltAngleRef * macros.D2R
    )
    interpolatedMotor1Angle = macros.R2D * interpolatedAngles.angle1
    interpolatedMotor2Angle = macros.R2D * interpolatedAngles.angle2
    simIsValidInterpolation = interpolatedAngles.isValidInterpolation

    # Calculate truth data
    truthMotor1Angle, truthMotor2Angle, truthIsValidInterpolation = gimbal_to_motor_angles(
        gimbalTipAngleRef, gimbalTiltAngleRef
    )

    # Assert: Check the truth values match the obtained results
    np.testing.assert_allclose(interpolatedMotor1Angle, truthMotor1Angle, atol=1e-4, verbose=True)
    np.testing.assert_allclose(interpolatedMotor2Angle, truthMotor2Angle, atol=1e-4, verbose=True)
    np.testing.assert_equal(simIsValidInterpolation, truthIsValidInterpolation)


def test_twoAxisGimbalLookupTables_gimbalToMotorAngles_linearInterpolation():
    # Load the gimbal interpolation tables
    (
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    ) = load_interpolation_tables()

    # Create the gimbal lookup table class
    gimbalLookupTables = twoAxisGimbal.TwoAxisGimbalLookupTables(
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    )

    # Set the gimbal reference angles
    gimbalTipAngleRef = 0.0  # [deg]
    gimbalTiltAngleRef = 0.4  # [deg]

    # Interpolate the motor angles using the gimbal lookup table module
    interpolatedAngles = gimbalLookupTables.gimbalAnglesToMotorAngles(
        gimbalTipAngleRef * macros.D2R, gimbalTiltAngleRef * macros.D2R
    )
    interpolatedMotor1Angle = macros.R2D * interpolatedAngles.angle1
    interpolatedMotor2Angle = macros.R2D * interpolatedAngles.angle2
    simIsValidInterpolation = interpolatedAngles.isValidInterpolation

    # Calculate truth data
    truthMotor1Angle, truthMotor2Angle, truthIsValidInterpolation = gimbal_to_motor_angles(
        gimbalTipAngleRef, gimbalTiltAngleRef
    )

    # Assert: Check the truth values match the obtained results
    np.testing.assert_allclose(interpolatedMotor1Angle, truthMotor1Angle, atol=1e-4, verbose=True)
    np.testing.assert_allclose(interpolatedMotor2Angle, truthMotor2Angle, atol=1e-4, verbose=True)
    np.testing.assert_equal(simIsValidInterpolation, truthIsValidInterpolation)


def test_twoAxisGimbalLookupTables_gimbalToMotorAngles_bilinearInterpolation():
    # Load the gimbal interpolation tables
    (
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    ) = load_interpolation_tables()

    # Create the gimbal lookup table class
    gimbalLookupTables = twoAxisGimbal.TwoAxisGimbalLookupTables(
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    )

    # Set the gimbal reference angles
    gimbalTipAngleRef = 0.4  # [deg]
    gimbalTiltAngleRef = 0.4  # [deg]

    # Interpolate the motor angles using the gimbal lookup table module
    interpolatedAngles = gimbalLookupTables.gimbalAnglesToMotorAngles(
        gimbalTipAngleRef * macros.D2R, gimbalTiltAngleRef * macros.D2R
    )
    interpolatedMotor1Angle = macros.R2D * interpolatedAngles.angle1
    interpolatedMotor2Angle = macros.R2D * interpolatedAngles.angle2
    simIsValidInterpolation = interpolatedAngles.isValidInterpolation

    # Calculate truth data
    truthMotor1Angle, truthMotor2Angle, truthIsValidInterpolation = gimbal_to_motor_angles(
        gimbalTipAngleRef, gimbalTiltAngleRef
    )

    # Assert: Check the truth values match the obtained results
    np.testing.assert_allclose(interpolatedMotor1Angle, truthMotor1Angle, atol=1e-4, verbose=True)
    np.testing.assert_allclose(interpolatedMotor2Angle, truthMotor2Angle, atol=1e-4, verbose=True)
    np.testing.assert_equal(simIsValidInterpolation, truthIsValidInterpolation)


def test_twoAxisGimbalLookupTables_gimbalToMotorAngles_invalidInterpolation():
    # Load the gimbal interpolation tables
    (
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    ) = load_interpolation_tables()

    # Create the gimbal lookup table class
    gimbalLookupTables = twoAxisGimbal.TwoAxisGimbalLookupTables(
        gimbal_to_motor_1_angle_table,
        gimbal_to_motor_2_angle_table,
        motor_to_gimbal_tip_angle_table,
        motor_to_gimbal_tilt_angle_table,
    )

    # Set the gimbal reference angles
    gimbalTipAngleRef = -10.2  # [deg]
    gimbalTiltAngleRef = -11.75  # [deg]

    # Interpolate the motor angles using the gimbal lookup table module
    interpolatedAngles = gimbalLookupTables.gimbalAnglesToMotorAngles(
        gimbalTipAngleRef * macros.D2R, gimbalTiltAngleRef * macros.D2R
    )
    simIsValidInterpolation = interpolatedAngles.isValidInterpolation

    # Calculate truth data
    truthMotor1Angle, truthMotor2Angle, truthIsValidInterpolation = gimbal_to_motor_angles(
        gimbalTipAngleRef, gimbalTiltAngleRef
    )

    # Assert: Check that the interpolation was flagged as invalid
    np.testing.assert_equal(simIsValidInterpolation, 0)
    np.testing.assert_equal(simIsValidInterpolation, truthIsValidInterpolation)

if __name__ == "__main__":
    test_twoAxisGimbalLookupTables_gimbalToMotorAngles_noInterpolation()
    test_twoAxisGimbalLookupTables_gimbalToMotorAngles_linearInterpolation()
    test_twoAxisGimbalLookupTables_gimbalToMotorAngles_bilinearInterpolation()
    test_twoAxisGimbalLookupTables_gimbalToMotorAngles_invalidInterpolation()
