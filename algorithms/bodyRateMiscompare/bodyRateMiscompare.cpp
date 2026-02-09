#include "bodyRateMiscompare.h"

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "msgPayloadDef/STAttMsgF32Payload.h"

template <size_t N>
void convertArray(const double (&src)[N], float (&dst)[N]) {
    for (size_t i = 0; i < N; ++i) dst[i] = static_cast<float>(src[i]);
}

inline void convert(const STAttMsgPayload& src, STAttMsgF32Payload& dst) {
    dst.timeTag = static_cast<float>(src.timeTag);
    convertArray(src.MRP_BdyInrtl, dst.MRP_BdyInrtl);
    convertArray(src.omega_BN_B, dst.omega_BN_B);
    convertArray(src.dcm_CB, dst.dcm_CB);
}

void BodyRateMiscompare::reset(uint64_t const callTime) {
    if (!this->imuSensorBodyInMsg.isLinked()) {
        throw std::invalid_argument("The imuSensorBodyInMsg was not linked and is required for execution");
    }
    if (!this->stBodyInMsg.isLinked()) {
        throw std::invalid_argument("The stSensInMsg was not linked and is required for execution");
    }
}

void BodyRateMiscompare::updateState(uint64_t const callTime) {
    // Retrieve the updated messages from the imuPayload and star tracker
    IMUSensorBodyMsgF32Payload imuPayload = this->imuSensorBodyInMsg();

    STAttMsgF32Payload stAttMsgF32Payload{};
    convert(this->stBodyInMsg(), stAttMsgF32Payload);

    // Call the algorithm to get the measured body rates
    auto [omega_BN_B, bodyRateFaultDetected] = this->algorithm.update(
        Eigen::Map<Eigen::Vector3f>(imuPayload.AngVelBody), Eigen::Map<Eigen::Vector3f>(stAttMsgF32Payload.omega_BN_B));

    NavAttMsgF32Payload navAttMsgPayload{};
    eigenVectorToCArray(omega_BN_B, navAttMsgPayload.omega_BN_B);
    navAttMsgPayload.timeTag = static_cast<double>(callTime) * NANO2SEC;

    BodyRateFaultMsgPayload bodyRateFaultPayload{};
    bodyRateFaultPayload.faultDetected = bodyRateFaultDetected;

    this->navAttOutMsg.write(&navAttMsgPayload, this->moduleID, callTime);
    this->rateFaultOutMsg.write(&bodyRateFaultPayload, this->moduleID, callTime);
}

void BodyRateMiscompare::setBodyRateThreshold(double const bodyRateThreshold) {
    this->algorithm.setBodyRateThreshold(static_cast<float>(bodyRateThreshold));
}

double BodyRateMiscompare::getBodyRateThreshold() const { return this->algorithm.getBodyRateThreshold(); }
