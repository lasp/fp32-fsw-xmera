#include "bodyRateMiscompare.h"

#include "msgPayloadDef/STAttMsgF32Payload.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/timeConstants.h"
#include "utilities/xmera/xmeraLifecycleException.h"

template <size_t N>
static void convertArray(const double (&src)[N], float (&dst)[N]) {
    for (size_t i = 0; i < N; ++i) {
        dst[i] = static_cast<float>(src[i]);
    }
}

inline void convert(const STAttMsgPayload& src, STAttMsgF32Payload& dst) {
    dst.timeTag = static_cast<float>(src.timeTag);
    convertArray(src.MRP_BdyInrtl, dst.MRP_BdyInrtl);
    convertArray(src.omega_BN_B, dst.omega_BN_B);
    convertArray(src.dcm_CB, dst.dcm_CB);
}

/*! This method performs a complete reset of the module. Validates that input messages are linked and resets the
 algorithm state.
 @return void
 @param callTime The clock time at which the function was called [nanoseconds]
 */
void BodyRateMiscompare::reset(uint64_t const callTime) {
    if (!this->imuSensorBodyInMsg.isLinked()) {
        throw std::invalid_argument("The imuSensorBodyInMsg was not linked and is required for execution");
    }
    if (!this->stBodyInMsg.isLinked()) {
        throw std::invalid_argument("The stSensInMsg was not linked and is required for execution");
    }

    auto config =
        BodyRateMiscompareConfig::create(this->bodyRateThreshold, this->faultPersistenceLimit, this->useImuRates);
    this->algorithm = std::make_unique<BodyRateMiscompareAlgorithm>(config);
}

/*! This method reads the IMU and star tracker messages, calls the body rate miscompare algorithm, and writes the
 output messages.
 @return void
 @param callTime The clock time at which the function was called [nanoseconds]
 */
void BodyRateMiscompare::updateState(uint64_t const callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("BodyRateMiscompare reset() has not been called.");
    }

    // Retrieve the updated messages from the imuPayload and star tracker
    IMUSensorBodyMsgF32Payload imuPayload = this->imuSensorBodyInMsg();

    STAttMsgF32Payload stAttMsgF32Payload{};
    convert(this->stBodyInMsg(), stAttMsgF32Payload);

    // Call the algorithm to get the measured body rates
    auto [omega_BN_B, bodyRateFaultDetected] = this->algorithm->update(
        cArrayToEigenVector(imuPayload.AngVelBody), cArrayToEigenVector(stAttMsgF32Payload.omega_BN_B));

    NavAttMsgF32Payload navAttMsgPayload{};
    eigenVectorToCArray(omega_BN_B, navAttMsgPayload.omega_BN_B);
    navAttMsgPayload.timeTag = static_cast<double>(callTime) * kNano2Sec;

    BodyRateFaultMsgPayload bodyRateFaultPayload{};
    bodyRateFaultPayload.faultDetected = bodyRateFaultDetected;

    this->navAttOutMsg.write(navAttMsgPayload, this->moduleID, callTime);
    this->rateFaultOutMsg.write(bodyRateFaultPayload, this->moduleID, callTime);
}

/*! @brief Clear the algorithm's persistence counter, preserving any latched fault. */
void BodyRateMiscompare::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("BodyRateMiscompare reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! @brief Fully reset the algorithm state: clear the persistence counter and the latched fault. */
void BodyRateMiscompare::reInitializeAll() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("BodyRateMiscompare reset() has not been called.");
    }
    this->algorithm->reInitializeAll();
}
