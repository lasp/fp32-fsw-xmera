// NOLINTBEGIN
#include "bodyRateMiscompare.h"

#include "architecture/utilities/macroDefinitions.h"
#include "msgPayloadDef/STAttMsgF32Payload.h"

template <size_t N>
void convertArray(const double (&src)[N], float (&dst)[N]) {
    for (size_t i = 0; i < N; ++i) dst[i] = static_cast<float>(src[i]);
}

template <size_t N>
void convertArray(const float (&src)[N], double (&dst)[N]) {
    for (size_t i = 0; i < N; ++i) dst[i] = static_cast<double>(src[i]);
}

inline void convert(const STAttMsgPayload& src, STAttMsgF32Payload& dst) {
    dst.timeTag = static_cast<float>(src.timeTag);
    convertArray(src.MRP_BdyInrtl, dst.MRP_BdyInrtl);
    convertArray(src.omega_BN_B, dst.omega_BN_B);
    convertArray(src.dcm_CB, dst.dcm_CB);
}

inline void convert(const NavAttMsgF32Payload& src, NavAttMsgPayload& dst) {
    dst.timeTag = static_cast<double>(src.timeTag);
    convertArray(src.sigma_BN, dst.sigma_BN);
    convertArray(src.omega_BN_B, dst.omega_BN_B);
    convertArray(src.vehSunPntBdy, dst.vehSunPntBdy);
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
    const IMUSensorBodyMsgF32Payload imuPayload = this->imuSensorBodyInMsg();
    STAttMsgF32Payload stAttMsgF32Payload{};
    convert(this->stBodyInMsg(), stAttMsgF32Payload);

    // Call the algorithm to get the measured body rates
    auto [navAttMsgF32Payload, bodyRateFaultMsgPayload] =
        this->algorithm.update(callTime, imuPayload, stAttMsgF32Payload);

    NavAttMsgPayload navAttMsgPayload{};
    convert(navAttMsgF32Payload, navAttMsgPayload);
    this->navAttMsg.write(&navAttMsgPayload, this->moduleID, callTime);
    this->rateFaultMsg.write(&bodyRateFaultMsgPayload, this->moduleID, callTime);
}

void BodyRateMiscompare::setBodyRateThreshold(double const bodyRateThreshold) {
    this->algorithm.setBodyRateThreshold(static_cast<float>(bodyRateThreshold));
}

double BodyRateMiscompare::getBodyRateThreshold() const { return this->algorithm.getBodyRateThreshold(); }
// NOLINTEND