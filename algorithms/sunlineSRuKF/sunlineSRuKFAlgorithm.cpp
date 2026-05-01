#include "sunlineSRuKFAlgorithm.h"

SunlineSRuKFOutput SunlineSRuKFAlgorithm::updateState(const SunlineSRuKFInput& input) {
    SunlineSRuKFOutput output{};
    output.timeTag = input.timeTag;
    output.sigma_BN = input.sigma_BN;
    output.omega_BN_B = input.omega_BN_B;
    output.vehSunPntBdy = input.vehSunPntBdy;
    return output;
}
