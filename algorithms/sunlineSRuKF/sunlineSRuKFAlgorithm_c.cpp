/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics,
 University of Colorado at Boulder
 */

#include "sunlineSRuKFAlgorithm_c.h"
#include "sunlineSRuKFAlgorithm.h"

#include <Eigen/Core>

uint32_t SunlineSRuKFAlgorithm_getMaxNumCss(void) { return SUNLINE_SRUKF_MAX_NUM_CSS; }

SunlineSRuKFOutput_c SunlineSRuKFAlgorithm_updateState(const SunlineSRuKFInput_c* input) {
    // Convert C POD input to C++ input
    SunlineSRuKFInput cppInput{};
    cppInput.timeTag = input->timeTag;
    cppInput.sigma_BN << input->sigma_BN.data[0], input->sigma_BN.data[1], input->sigma_BN.data[2];
    cppInput.omega_BN_B << input->omega_BN_B.data[0], input->omega_BN_B.data[1], input->omega_BN_B.data[2];
    cppInput.vehSunPntBdy << input->vehSunPntBdy.data[0], input->vehSunPntBdy.data[1], input->vehSunPntBdy.data[2];
    cppInput.nCSS = input->nCSS;
    for (uint32_t i = 0; i < SUNLINE_SRUKF_MAX_NUM_CSS; ++i) {
        cppInput.cosValues[i] = input->cosValues[i];
    }

    // Call static algorithm method
    const SunlineSRuKFOutput cppOutput = ::SunlineSRuKFAlgorithm::updateState(cppInput);

    // Convert C++ output to C POD output
    SunlineSRuKFOutput_c output{};
    output.timeTag = cppOutput.timeTag;
    output.sigma_BN.data[0] = cppOutput.sigma_BN[0];
    output.sigma_BN.data[1] = cppOutput.sigma_BN[1];
    output.sigma_BN.data[2] = cppOutput.sigma_BN[2];
    output.omega_BN_B.data[0] = cppOutput.omega_BN_B[0];
    output.omega_BN_B.data[1] = cppOutput.omega_BN_B[1];
    output.omega_BN_B.data[2] = cppOutput.omega_BN_B[2];
    output.vehSunPntBdy.data[0] = cppOutput.vehSunPntBdy[0];
    output.vehSunPntBdy.data[1] = cppOutput.vehSunPntBdy[1];
    output.vehSunPntBdy.data[2] = cppOutput.vehSunPntBdy[2];

    return output;
}
