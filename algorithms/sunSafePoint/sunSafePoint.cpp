#include "sunSafePoint.h"

#include <architecture/utilities/eigenSupport.h>
#include <cassert>

/*! Reset method for the BSK module adapter interface. This method also calls the algorithm reset method.
 @return void
 @param callTime [ns] Time the method is called
*/
void SunSafePoint::reset(uint64_t callTime) {
    assert(this->sunDirectionInMsg.isLinked() && this->imuInMsg.isLinked());

    // Call the algorithm reset method
    this->algorithm.reset(callTime);
}

/*! Update method for the BSK module adapter interface. This method also calls the algorithm update method.
 @return void
 @param callTime [ns] Time the method is called
*/
void SunSafePoint::updateState(uint64_t callTime) {
    auto imuMsgPayload = NavAttMsgF32Payload();
    if (this->imuInMsg.isWritten()) {
        imuMsgPayload = this->imuInMsg();
    }
    auto sunDirectionMsgPayload = NavAttMsgF32Payload();
    if (this->sunDirectionInMsg.isWritten()) {
        sunDirectionMsgPayload = this->sunDirectionInMsg();
    }

    // Call the algorithm update method
    AttGuidMsgF32Payload attGuidanceOutBuffer = this->algorithm.update(callTime, imuMsgPayload, sunDirectionMsgPayload);

    this->attGuidanceOutMsg.write(&attGuidanceOutBuffer, moduleID, callTime);
}

/*! Getter method for the minimally accepted sun body vector norm.
 @return float
*/
float SunSafePoint::getMinUnitMag() const { return this->algorithm.getMinUnitMag(); }

/*! Getter method for the small alignment tolerance angle near 0 or 180 degrees.
 @return float
*/
float SunSafePoint::getSmallAngle() const { return this->algorithm.getSmallAngle(); }

/*! Getter method for the desired constant spin rate about sun heading vector.
 @return float
*/
float SunSafePoint::getSunAxisSpinRate() const { return this->algorithm.getSunAxisSpinRate(); }

/*! Getter method for the desired body rate vector if no sun direction is available.
 @return Eigen::Vector3f
*/
Eigen::Vector3f SunSafePoint::getOmega_RN_B() const { return this->algorithm.getOmega_RN_B(); }

/*! Getter method for the desired body vector to point at the sun.
 @return Eigen::Vector3f
*/
Eigen::Vector3f SunSafePoint::getSHatBdyCmd() const { return this->algorithm.getSHatBdyCmd(); }

/*! Setter method for the minimally accepted sun body vector norm.
 @return void
 @param minUnitMag The minimally acceptable norm of sun body vector
*/
void SunSafePoint::setMinUnitMag(const float minUnitMag) { this->algorithm.setMinUnitMag(minUnitMag); }

/*! Setter method for the small alignment tolerance angle near 0 or 180 degrees.
 @return void
 @param smallAngle [rad] An angle value that specifies what is near 0 or 180 degrees
*/
void SunSafePoint::setSmallAngle(const float smallAngle) { this->algorithm.setSmallAngle(smallAngle); }

/*! Setter method for the desired constant spin rate about sun heading vector.
 @return void
 @param sunAxisSpinRate [rad/s] Desired constant spin rate about sun heading vector
*/
void SunSafePoint::setSunAxisSpinRate(const float sunAxisSpinRate) {
    this->algorithm.setSunAxisSpinRate(sunAxisSpinRate);
}

/*! Setter method for the desired body rate vector if no sun direction is available.
 @return void
 @param omega_RN_B [rad/s] Desired body rate vector if no sun direction is available
*/
void SunSafePoint::setOmega_RN_B(const Eigen::Vector3f& omega_RN_B) { this->algorithm.setOmega_RN_B(omega_RN_B); }

/*! Setter method for the desired body vector to point at the sun.
 @return void
 @param sHatBdyCmd Desired body vector to point at the sun
*/
void SunSafePoint::setSHatBdyCmd(Eigen::Vector3f& sHatBdyCmd) { this->algorithm.setSHatBdyCmd(sHatBdyCmd); }
