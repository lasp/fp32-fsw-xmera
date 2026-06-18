#include "bodyRateMiscompare.h"

#include "msgPayloadDef/STAttMsgF32Payload.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/timeConstants.h"

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

    this->algorithm.reset();
}

/*! This method reads the IMU and star tracker messages, calls the body rate miscompare algorithm, and writes the
 output messages.
 @return void
 @param callTime The clock time at which the function was called [nanoseconds]
 */
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
    navAttMsgPayload.timeTag = static_cast<double>(callTime) * kNano2Sec;

    BodyRateFaultMsgPayload bodyRateFaultPayload{};
    bodyRateFaultPayload.faultDetected = bodyRateFaultDetected;

    this->navAttOutMsg.write(&navAttMsgPayload, this->moduleID, callTime);
    this->rateFaultOutMsg.write(&bodyRateFaultPayload, this->moduleID, callTime);
}

/*! Setter method for bodyRateThreshold. Converts from double to float for the algorithm.
 @return void
 @param bodyRateThreshold [rad/s] threshold for rate miscompare detection
 */
void BodyRateMiscompare::setBodyRateThreshold(double const bodyRateThreshold) {
    this->algorithm.setBodyRateThreshold(static_cast<float>(bodyRateThreshold));
}

/*! Getter method for bodyRateThreshold.
 @return double
 */
double BodyRateMiscompare::getBodyRateThreshold() const { return this->algorithm.getBodyRateThreshold(); }

/*! Setter method for faultPersistenceLimit.
 @return void
 @param faultPersistenceLimit number of consecutive threshold violations before fault is declared
 */
void BodyRateMiscompare::setFaultPersistenceLimit(uint32_t const faultPersistenceLimit) {
    this->algorithm.setFaultPersistenceLimit(faultPersistenceLimit);
}

/*! Getter method for faultPersistenceLimit.
 @return uint32_t
 */
uint32_t BodyRateMiscompare::getFaultPersistenceLimit() const { return this->algorithm.getFaultPersistenceLimit(); }

/*! Setter method for useImuRates.
 @return void
 @param useImuRates flag to force IMU rate output
 */
void BodyRateMiscompare::setUseImuRates(bool const useImuRates) { this->algorithm.setUseImuRates(useImuRates); }

/*! Getter method for useImuRates.
 @return bool
 */
bool BodyRateMiscompare::getUseImuRates() const { return this->algorithm.getUseImuRates(); }
