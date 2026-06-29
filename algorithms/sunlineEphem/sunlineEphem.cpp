#include "sunlineEphem.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
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

    auto config = SunlineEphemConfig::create();
    this->algorithm = std::make_unique<SunlineEphemAlgorithm>(config);
}

void SunlineEphem::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SunlineEphem reset() has not been called.");
    }

    EphemerisMsgF32Payload sunPosition = this->sunPositionInMsg();
    NavTransMsgF32Payload spacecraftPosition = this->scPositionInMsg();
    NavAttMsgF32Payload spacecraftAttitude = this->scAttitudeInMsg();

    const Eigen::Vector3d r_SN_N = cArrayToEigenVector(sunPosition.r_BdyZero_N);
    const Eigen::Vector3d r_BN_N = cArrayToEigenVector(spacecraftPosition.r_BN_N);
    const Eigen::Vector3f sigma_BN = cArrayToEigenVector(spacecraftAttitude.sigma_BN);

    const Eigen::Vector3f rHat_SB_B = this->algorithm->update(r_SN_N, r_BN_N, sigma_BN);

    NavAttMsgF32Payload outputSunline{};
    eigenVectorToCArray(rHat_SB_B, outputSunline.vehSunPntBdy);
    this->navStateOutMsg.write(outputSunline, this->moduleID, callTime);
}
