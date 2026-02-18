#include "sunlineEphem.h"

#include <architecture/utilities/eigenSupport.h>
#include <stdexcept>

void SunlineEphem::reset(uint64_t callTime) {
    if (!this->sunPositionInMsg.isLinked()) {
        throw std::invalid_argument("SunlineEphem.sunPositionInMsg is unlinked");
    }
    if (!this->scPositionInMsg.isLinked()) {
        throw std::invalid_argument("SunlineEphem.scPositionInMsg is unlinked");
    }
    if (!this->scAttitudeInMsg.isLinked()) {
        throw std::invalid_argument("SunlineEphem.scAttitudeInMsg is unlinked");
    }
}

void SunlineEphem::updateState(uint64_t callTime) {
    EphemerisMsgF32Payload sunPos = this->sunPositionInMsg();
    NavTransMsgF32Payload scPos = this->scPositionInMsg();
    NavAttMsgF32Payload scAtt = this->scAttitudeInMsg();

    const Eigen::Vector3d rSun = cArrayToEigenVector(sunPos.r_BdyZero_N);
    const Eigen::Vector3d rSc = cArrayToEigenVector(scPos.r_BN_N);
    const Eigen::Vector3f sigma_BN = cArrayToEigenVector(scAtt.sigma_BN);

    const Eigen::Vector3f vehSunPntBdy = this->algorithm.updateState(rSun, rSc, sigma_BN);

    NavAttMsgF32Payload outputSunline{};
    eigenVectorToCArray(vehSunPntBdy, outputSunline.vehSunPntBdy);
    this->navStateOutMsg.write(&outputSunline, this->moduleID, callTime);
}
