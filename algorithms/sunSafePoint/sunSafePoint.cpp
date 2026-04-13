#include "sunSafePoint.h"

#include <architecture/utilities/eigenSupport.h>
#include <stdexcept>

/*! Reset method for the BSK module adapter interface. This method also calls the algorithm reset method.
 @return void
 @param callTime [ns] Time the method is called
*/
void SunSafePoint::reset(uint64_t callTime) {
    if (!this->sunDirectionInMsg.isLinked()) {
        throw std::invalid_argument("sunSafePoint.sunDirectionInMsg wasn't connected.");
    }
    if (!this->imuInMsg.isLinked()) {
        throw std::invalid_argument("sunSafePoint.imuInMsg wasn't connected.");
    }

    // Call the algorithm reset method
    this->algorithm.reset();
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

    Eigen::Vector3f const vehSunPntBdy = cArrayToEigenVector(sunDirectionMsgPayload.vehSunPntBdy);
    Eigen::Vector3f const omega_BN_B = cArrayToEigenVector(imuMsgPayload.omega_BN_B);

    // Call the algorithm update method
    SunSafePointOutput output = this->algorithm.update(vehSunPntBdy, omega_BN_B);

    // Convert algorithm output to MsgPayload
    AttGuidMsgF32Payload attGuidanceOutBuffer{};
    eigenVectorToCArray(output.sigma_BR, attGuidanceOutBuffer.sigma_BR);
    eigenVectorToCArray(output.omega_BR_B, attGuidanceOutBuffer.omega_BR_B);
    eigenVectorToCArray(output.omega_RN_B, attGuidanceOutBuffer.omega_RN_B);

    this->attGuidanceOutMsg.write(&attGuidanceOutBuffer, moduleID, callTime);
}

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

/*! Setter method for the small alignment tolerance angle near 0 or 180 degrees.
 @return void
 @param angle [rad] An angle value that specifies what is near 0 or 180 degrees
*/
void SunSafePoint::setSmallAngle(const float angle) { this->algorithm.setSmallAngle(angle); }

/*! Setter method for the desired constant spin rate about sun heading vector.
 @return void
 @param rate [rad/s] Desired constant spin rate about sun heading vector
*/
void SunSafePoint::setSunAxisSpinRate(const float rate) { this->algorithm.setSunAxisSpinRate(rate); }

/*! Setter method for the desired body rate vector if no sun direction is available.
 @return void
 @param omega [rad/s] Desired body rate vector if no sun direction is available
*/
void SunSafePoint::setOmega_RN_B(const Eigen::Vector3f& omega) { this->algorithm.setOmega_RN_B(omega); }

/*! Setter method for the desired body vector to point at the sun.
 @return void
 @param sHat Desired body vector to point at the sun
*/
void SunSafePoint::setSHatBdyCmd(const Eigen::Vector3f& sHat) { this->algorithm.setSHatBdyCmd(sHat); }
