#ifndef F32XMERA_ALGORITHM_C_SHIM_TYPES_H
#define F32XMERA_ALGORITHM_C_SHIM_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

/**
 * @brief POD representation of a 3x3 matrix (Eigen::Matrix3f).
 *
 * Stored in row-major order: data[row][col].
 */
typedef struct {
    float data[3][3];
} Matrix3f_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_ALGORITHM_C_SHIM_TYPES_H
