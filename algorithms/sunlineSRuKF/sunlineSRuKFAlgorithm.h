/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_SUNLINE_SRUKF_ALGORITHM_H
#define F32XIMERA_SUNLINE_SRUKF_ALGORITHM_H

#include <Eigen/Core>
#include <cstdint>

/*! @brief Input structure for the sunline SRuKF pass-through algorithm. */
struct SunlineSRuKFInput {
    double timeTag{};
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f vehSunPntBdy = Eigen::Vector3f::Zero();
    uint32_t nCSS{};        //!< [-] Number of coarse sun sensors
    float cosValues[32]{};  //!< [-] CSS cosine measurement values
};

/*! @brief Output structure for the sunline SRuKF pass-through algorithm. */
struct SunlineSRuKFOutput {
    double timeTag{};
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f vehSunPntBdy = Eigen::Vector3f::Zero();
};

/*! @brief Sunline SRuKF pass-through algorithm class. */
class SunlineSRuKFAlgorithm {
   public:
    static SunlineSRuKFOutput updateState(const SunlineSRuKFInput& input);
};

#endif
