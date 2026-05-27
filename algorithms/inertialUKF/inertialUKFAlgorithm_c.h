#ifndef F32XMERA_INERTIALUKFALGORITHM_C_H
#define F32XMERA_INERTIALUKFALGORITHM_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C-compatible representation of STAttInput.
 */
typedef struct {
    float timeTag;         /*!< Time tag of star tracker measurement (s) */
    float MRP_BdyInrtl[3]; /*!< MRP attitude of body w.r.t. inertial frame */
    float omega_BN_B[3];   /*!< Angular velocity of body w.r.t. inertial, body frame (rad/s) */
} STAttInput_c;

/**
 * @brief C-compatible representation of GyroInput.
 */
typedef struct {
    float gyro_B[3]; /*!< Gyro measurement in body frame (rad/s) */
} GyroInput_c;

/**
 * @brief C-compatible representation of RWSpeedsInput.
 */
typedef struct {
    float wheelSpeeds[4]; /*!< Reaction wheel speeds (rad/s) */
} RWSpeedsInput_c;

/**
 * @brief C-compatible representation of RWArrayConfigInput.
 */
typedef struct {
    int numRW; /*!< Number of active reaction wheels */
} RWArrayConfigInput_c;

/**
 * @brief C-compatible representation of VehicleConfigInput.
 *
 * ISCPntB_B is stored in column-major order to match the Eigen::Matrix3f
 * memory layout used internally.
 */
typedef struct {
    float ISCPntB_B[9]; /*!< Inertia tensor about SC body point B, body frame (kg*m^2), column-major */
    float massSC;       /*!< Spacecraft mass (kg) */
} VehicleConfigInput_c;

/**
 * @brief C-compatible representation of NavAttOutput.
 */
typedef struct {
    double timeTag;        /*!< Time tag of navigation solution (s) */
    float sigma_BN[3];     /*!< MRP attitude of body w.r.t. inertial frame */
    float omega_BN_B[3];   /*!< Angular velocity of body w.r.t. inertial, body frame (rad/s) */
    float vehSunPntBdy[3]; /*!< Sun direction unit vector in body frame */
} NavAttOutput_c;

/**
 * @brief C-compatible representation of InertialFilterOutput.
 */
typedef struct {
    double timeTag; /*!< Time tag of filter solution (s) */
    int numObs;     /*!< Number of observations used in filter update */
} InertialFilterOutput_c;

/**
 * @brief C-compatible representation of InertialUKFOutput.
 */
typedef struct {
    NavAttOutput_c navAtt;         /*!< Navigation attitude estimate */
    InertialFilterOutput_c filter; /*!< Filter diagnostic data */
} InertialUKFOutput_c;

/**
 * @brief Run the inertial UKF algorithm update step.
 * @param stAtt     Pointer to star tracker attitude input.
 * @param gyro      Pointer to gyro measurement input.
 * @param rwSpeeds  Pointer to reaction wheel speeds input.
 * @param rwConfig  Pointer to reaction wheel array configuration input.
 * @param vehConfig Pointer to vehicle configuration input.
 * @return InertialUKFOutput_c  The combined navigation and filter output.
 */
InertialUKFOutput_c InertialUKFAlgorithm_updateState(const STAttInput_c* stAtt,
                                                     const GyroInput_c* gyro,
                                                     const RWSpeedsInput_c* rwSpeeds,
                                                     const RWArrayConfigInput_c* rwConfig,
                                                     const VehicleConfigInput_c* vehConfig);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_INERTIALUKFALGORITHM_C_H
