/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _TWO_AXIS_GIMBAL_LOOKUP_TABLES_
#define _TWO_AXIS_GIMBAL_LOOKUP_TABLES_

#include <array>
#include <numbers>

#define NUM_GIMBAL_TO_MOTOR_TABLE_ROWS 111
#define NUM_GIMBAL_TO_MOTOR_TABLE_COLS 76

const double DEG2RAD = M_PI / 180.0;

enum class FixedAngle { ANGLE_1_FIXED, ANGLE_2_FIXED };

struct MotorAngles {
    double angle1;
    double angle2;
    bool isValidInterpolation;
};

/*! @brief Two Axis Gimbal Lookup Table Class */
class TwoAxisGimbalLookupTables {
   public:
    MotorAngles gimbalAnglesToMotorAngles(double gimbalTipAngle,
                                          double gimbalTiltAngle);  //!< Method to determine the stepper motor angles
                                                                    //!< given the gimbal sequential tip and tilt angles

   private:
    MotorAngles pullAngles(double angle1, double angle2) const;
    bool bilinearInterpolationRequired(double angle1,
                                       double angle2);  //!< Method to determine if bilinear interpolation is required
    bool noInterpolationRequired(double angle1,
                                 double angle2);     //!< Method to determine if no interpolation is required
    bool linearInterpolationRequired(double angle);  //!< Method to determine if linear interpolation is required
    MotorAngles bilinearlyInterpolateAngles(double angle1, double angle2);
    MotorAngles linearlyInterpolateAngles(double angle1, double angle2, FixedAngle fixedAngle);

    double tableStepAngle{0.5 * DEG2RAD};  //!< [rad] Interpolation table motor discretization angle
    std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        gimbalAnglesToMotor1AngleData;  //!< [rad] Gimbal-to-motor 1 angle interpolation table storage array
    std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        gimbalAnglesToMotor2AngleData;  //!< [rad] Gimbal-to-motor 2 angle interpolation table storage array
};

#endif
