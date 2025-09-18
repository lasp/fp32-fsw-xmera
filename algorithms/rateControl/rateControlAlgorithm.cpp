/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "rateControl.h"

/*! This function performs the conversion between an input C array
3-vector and an output Eigen vector3f. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::Vector3f
@param inArray The input array (row-major)
*/
static Eigen::Vector3f cArray2EigenVector3f(float *inArray) { return Eigen::Map<Eigen::Vector3f>(inArray, 3, 1); }

/*! This function provides a direct conversion between a 3-vector and an
output C array. We are providing this function to save on the  inline conversion
and the transpose that would have been performed by the general case.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array we copy into
*/
static void eigenVector3f2CArray(Eigen::Vector3f &inMat, float *outArray) {
    memcpy(outArray, inMat.data(), 3 * sizeof(float));
}

/*! This function performs the general conversion between an input C array
and an Eigen matrix. Note that to use this function the user MUST size
the Eigen matrix ahead of time so that the internal map call has enough
information to ingest the C array.
@return Eigen::MatrixXf
@param inArray The input array (row-major)
@param nRows
@param nCols
*/
static Eigen::MatrixXf cArray2EigenMatrixXf(float *inArray, int nRows, int nCols) {
    Eigen::MatrixXf outMat;
    outMat.resize(nRows, nCols);
    outMat = Eigen::Map<Eigen::MatrixXf>(inArray, outMat.rows(), outMat.cols());
    return outMat;
}

/*! This method takes the attitude and rate errors relative to the reference frame, as well as
the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return torqueCmdOut
 @param attGuidIn Attitude guidance input
*/
CmdTorqueBodyMsgF32Payload RateControlAlgorithm::update(AttGuidMsgF32Payload attGuidIn) {
    CmdTorqueBodyMsgF32Payload torqueCmdOut{};

    // Compute required attitude control torque vector
    Eigen::Vector3f omega_BR_B = cArray2EigenVector3f(attGuidIn.omega_BR_B);
    Eigen::Vector3f omega_RN_B = cArray2EigenVector3f(attGuidIn.omega_RN_B);
    Eigen::Vector3f omega_BN_B = omega_BR_B + omega_RN_B;
    Eigen::Vector3f domega_RN_B = cArray2EigenVector3f(attGuidIn.domega_RN_B);
    Eigen::Vector3f Lr = -this->P * omega_BR_B + omega_RN_B.cross(this->ISCPntB_B * omega_BN_B) +
                         this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B)) -
                         this->knownTorquePntB_B;  // [Nm]

    eigenVector3f2CArray(Lr, torqueCmdOut.torqueRequestBody);

    return torqueCmdOut;
}

/*! This method sets the spacecraft inertia according to the vehicle configuration input message
 @return void
 @param vehicleConfigIn Vehicle config input
*/
void RateControlAlgorithm::setSpacecraftInertia(VehicleConfigMsgF32Payload vehicleConfigIn) {
    this->ISCPntB_B = cArray2EigenMatrixXf(vehicleConfigIn.ISCPntB_B, 3, 3);
}

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void RateControlAlgorithm::setDerivativeGainP(const float P) {
    if (P < 0.0) {
        throw std::invalid_argument("Feedback gain P must not be negative");
    }
    this->P = P;
}

/*! Getter method for the derivative gain P.
 @return const float
*/
float RateControlAlgorithm::getDerivativeGainP() const { return this->P; }

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void RateControlAlgorithm::setKnownTorquePntB_B(const Eigen::Vector3f &knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3f
*/
const Eigen::Vector3f &RateControlAlgorithm::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
