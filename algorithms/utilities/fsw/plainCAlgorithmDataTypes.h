#ifndef FP32_XMERA_FSW_PLAIN_C_ALGORITHM_DATA_TYPES_H
#define FP32_XMERA_FSW_PLAIN_C_ALGORITHM_DATA_TYPES_H

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

/**
 * @brief POD representation of a 3x3 matrix (Eigen::Matrix3f).
 */
typedef struct {
    float data[3][3];
} Matrix3f_c;

#endif  // FP32_XMERA_FSW_PLAIN_C_ALGORITHM_DATA_TYPES_H
