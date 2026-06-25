#ifndef F32XMERA_TIME_CLOSEST_APPROACH_TYPES_H
#define F32XMERA_TIME_CLOSEST_APPROACH_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POD representation of a 3-vector of doubles (Eigen::Vector3d).
 */
typedef struct {
    double data[3];
} Vector3d_c;

/**
 * @brief POD output of the TCA algorithm.
 */
typedef struct {
    float tCA;      /*!< [s] Time of closest approach estimate */
    float sigmaTca; /*!< [s] Standard deviation of the TCA estimate */
} TimeClosestApproachOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_TIME_CLOSEST_APPROACH_TYPES_H
