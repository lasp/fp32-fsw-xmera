#include "sunlineFilter.h"

#include <utilities/fsw/eigenSupport.h>
#include <stdexcept>

void SunlineFilter::reset(uint64_t callTime) {
    if (!this->navAttInMsg.isLinked()) {
        throw std::invalid_argument("SunlineFilter.navAttInMsg is unlinked");
    }
    if (!this->cssDataInMsg.isLinked()) {
        throw std::invalid_argument("SunlineFilter.cssDataInMsg is unlinked");
    }
    if (!this->cssConfigInMsg.isLinked()) {
        throw std::invalid_argument("SunlineFilter.cssConfigInMsg is unlinked");
    }

    CSSConfigMsgPayload cssConfigBuffer = this->cssConfigInMsg();
    this->nCSS = cssConfigBuffer.nCSS;
}

void SunlineFilter::updateState(uint64_t callTime) {
    NavAttMsgF32Payload navAttIn = this->navAttInMsg();
    CSSArraySensorMsgPayload cssDataIn = this->cssDataInMsg();

    SunlineFilterInput input{};
    input.timeTag = navAttIn.timeTag;
    input.sigma_BN = cArrayToEigenVector3(navAttIn.sigma_BN);
    input.omega_BN_B = cArrayToEigenVector3(navAttIn.omega_BN_B);
    input.vehSunPntBdy = cArrayToEigenVector3(navAttIn.vehSunPntBdy);
    input.nCSS = this->nCSS;
    for (uint32_t i = 0; i < this->nCSS; ++i) {
        input.cosValues[i] = static_cast<float>(cssDataIn.CosValue[i]);
    }

    const SunlineFilterOutput output = SunlineFilterAlgorithm::updateState(input);

    NavAttMsgF32Payload navAttOut{};
    navAttOut.timeTag = output.timeTag;
    eigenVectorToCArray(output.sigma_BN, navAttOut.sigma_BN);
    eigenVectorToCArray(output.omega_BN_B, navAttOut.omega_BN_B);
    eigenVectorToCArray(output.vehSunPntBdy, navAttOut.vehSunPntBdy);

    this->navAttOutMsg.write(&navAttOut, this->moduleID, callTime);
}
