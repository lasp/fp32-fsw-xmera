#ifndef _TWO_AXIS_GIMBAL_LOOKUP_TABLES_
#define _TWO_AXIS_GIMBAL_LOOKUP_TABLES_

#include <array>
#include <numbers>

#define NUM_GIMBAL_TO_MOTOR_TABLE_ROWS 111
#define NUM_GIMBAL_TO_MOTOR_TABLE_COLS 76

const double DEG2RAD = M_PI / 180.0;

enum class InterpolationType { GIMBAL_ANGLES_TO_MOTOR_ANGLES, MOTOR_ANGLES_TO_GIMBAL_ANGLES };

enum class InterpolationTableType {
    GIMBAL_ANGLES_TO_MOTOR_1_ANGLES,
    GIMBAL_ANGLES_TO_MOTOR_2_ANGLES,
    MOTOR_ANGLES_TO_GIMBAL_TIP_ANGLES,
    MOTOR_ANGLES_TO_GIMBAL_TILT_ANGLES
};

enum class FixedAngle { ANGLE_1_FIXED, ANGLE_2_FIXED };

struct InterpolatedAngles {
    double angle1;
    double angle2;
    bool isValidInterpolation;
};

/*! @brief Two Axis Gimbal Lookup Table Class */
class TwoAxisGimbalLookupTables {
   public:
    TwoAxisGimbalLookupTables(const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>,
                                               NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>& gimbalToMotor1Data,
                              const std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>,
                                               NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>& gimbalToMotor2Data,
                              const std::array<std::array<double, 320>, 320>& motorToGimbalTipData,
                              const std::array<std::array<double, 320>, 320>& motorToGimbalTiltData);  //!< Constructor
    ~TwoAxisGimbalLookupTables() = default;                                                            //!< Destructor

    InterpolatedAngles gimbalAnglesToMotorAngles(
        double gimbalTipAngle,
        double gimbalTiltAngle);  //!< Method to determine the stepper motor angles given the gimbal
                                  //!< sequential tip and tilt angles
    InterpolatedAngles motorAnglesToGimbalAngles(
        double motor1Angle,
        double motor2Angle);  //!< Method to determine the sequential gimbal tip and tilt angles given the
                              //!< stepper motor angles

   private:
    double pullAngle(double angle1, double angle2, InterpolationTableType interpolationTableType) const;
    bool bilinearInterpolationRequired(double angle1,
                                       double angle2);  //!< Method to determine if bilinear interpolation is required
    bool noInterpolationRequired(double angle1,
                                 double angle2);     //!< Method to determine if no interpolation is required
    bool linearInterpolationRequired(double angle);  //!< Method to determine if linear interpolation is required
    InterpolatedAngles bilinearlyInterpolateAngles(double angle1, double angle2, InterpolationType interpolationType);
    InterpolatedAngles linearlyInterpolateAngles(double angle1,
                                                 double angle2,
                                                 InterpolationType interpolationType,
                                                 FixedAngle fixedAngle);

    double tableStepAngle{0.5 * DEG2RAD};  //!< [rad] Interpolation table motor discretization angle
    std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        gimbalAnglesToMotor1AngleData;  //!< [rad] Gimbal-to-motor 1 angle interpolation table storage array
    std::array<std::array<double, NUM_GIMBAL_TO_MOTOR_TABLE_COLS>, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>
        gimbalAnglesToMotor2AngleData;  //!< [rad] Gimbal-to-motor 2 angle interpolation table storage array
    std::array<std::array<double, 320>, 320>
        motorToGimbalTipAngleData;  //!< [rad] Motor-to-gimbal tip angle interpolation table storage array
    std::array<std::array<double, 320>, 320>
        motorToGimbalTiltAngleData;  //!< [rad] Motor-to-gimbal tilt angle interpolation table storage array
};

#endif
