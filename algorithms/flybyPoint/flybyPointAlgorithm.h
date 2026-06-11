// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_H

#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/FlybyDiagnosticMsgPayload.h>
#include <Eigen/Dense>

#include "flybyPointTypes.h"
#include "utilities/freestandingInvalidArgument.h"

class FlybyPointConfig final {
   public:
    /// Static factory — validates all parameters, throws on failure
    static FlybyPointConfig create(double timeBetweenFilterData,
                                   float toleranceForCollinearity,
                                   int signOfOrbitNormalFrameVector,
                                   float maxRateThreshold,
                                   float maxAccelerationThreshold,
                                   float positionKnowledgeSigma) {
        if (!isValidTimeBetweenFilterData(timeBetweenFilterData)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: timeBetweenFilterData must be >= 0");
        }
        if (!isValidToleranceForCollinearity(toleranceForCollinearity)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: toleranceForCollinearity must be >= 0");
        }
        if (!isValidSignOfOrbitNormalFrameVector(signOfOrbitNormalFrameVector)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: signOfOrbitNormalFrameVector must be 1 or -1");
        }
        if (!isValidMaxRateThreshold(maxRateThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: maxRateThreshold must be >= 0");
        }
        if (!isValidMaxAccelerationThreshold(maxAccelerationThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: maxAccelerationThreshold must be >= 0");
        }
        if (!isValidPositionKnowledgeSigma(positionKnowledgeSigma)) {
            FSW_THROW_INVALID_ARGUMENT("flybyPoint: positionKnowledgeSigma must be >= 0");
        }
        return {timeBetweenFilterData,
                toleranceForCollinearity,
                signOfOrbitNormalFrameVector,
                maxRateThreshold,
                maxAccelerationThreshold,
                positionKnowledgeSigma};
    }

    /// Static validators — only for parameters with a meaningful constraint
    static bool isValidTimeBetweenFilterData(double time) { return time >= 0.0; }
    static bool isValidToleranceForCollinearity(float tolerance) { return tolerance >= 0.0F; }
    static bool isValidSignOfOrbitNormalFrameVector(int sign) { return sign == 1 || sign == -1; }
    static bool isValidMaxRateThreshold(float maxRate) { return maxRate >= 0.0F; }
    static bool isValidMaxAccelerationThreshold(float maxAcceleration) { return maxAcceleration >= 0.0F; }
    static bool isValidPositionKnowledgeSigma(float sigma) { return sigma >= 0.0F; }

    /// Const getters — Config is immutable after construction
    double getTimeBetweenFilterData() const { return timeBetweenFilterData; }
    float getToleranceForCollinearity() const { return toleranceForCollinearity; }
    int getSignOfOrbitNormalFrameVector() const { return signOfOrbitNormalFrameVector; }
    float getMaxRateThreshold() const { return maxRateThreshold; }
    float getMaxAccelerationThreshold() const { return maxAccelerationThreshold; }
    float getPositionKnowledgeSigma() const { return positionKnowledgeSigma; }

   private:
    FlybyPointConfig(double timeBetweenFilterData,
                     float toleranceForCollinearity,
                     int signOfOrbitNormalFrameVector,
                     float maxRateThreshold,
                     float maxAccelerationThreshold,
                     float positionKnowledgeSigma)
        : timeBetweenFilterData(timeBetweenFilterData),
          toleranceForCollinearity(toleranceForCollinearity),
          signOfOrbitNormalFrameVector(signOfOrbitNormalFrameVector),
          maxRateThreshold(maxRateThreshold),
          maxAccelerationThreshold(maxAccelerationThreshold),
          positionKnowledgeSigma(positionKnowledgeSigma) {}

    double timeBetweenFilterData;
    float toleranceForCollinearity;
    int signOfOrbitNormalFrameVector;
    float maxRateThreshold;
    float maxAccelerationThreshold;
    float positionKnowledgeSigma;
};

/*! structure containing the attitude guidance outputs of the algorithm */
struct FlybyPointOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
    bool collinearityTrigger = false;
    bool maxRateTrigger = false;
    bool maxAccelerationTrigger = false;
    bool positionKnowledgeExceedTrigger = false;
};

/*! @brief A class to perform flyby pointing */
class FlybyPointAlgorithm final {
   public:
    explicit FlybyPointAlgorithm(const FlybyPointConfig& config);
    void setConfig(const FlybyPointConfig& config);

    void reset();
    FlybyPointOutput updateState(uint64_t currentSimNanos,
                                 const Eigen::Vector3d& r_BN_N,  //!< [m] relative position — double: §3.2 item 2
                                 const Eigen::Vector3d& v_BN_N   //!< [m/s] relative velocity — double: §3.2 item 2
    );
    bool checkValidityFirstRead(const Eigen::Vector3d& r_BN_N,
                                const Eigen::Vector3d& v_BN_N,
                                FlybyDiagnosticMsgPayload& flybyDiagnosticMsgBuffer) const;
    bool checkValidity(uint64_t currentSimNanos,
                       const Eigen::Vector3d& r_BN_N,
                       const Eigen::Vector3d& v_BN_N,
                       FlybyDiagnosticMsgPayload& flybyDiagnosticMsgBuffer) const;
    void computeFlybyParameters(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    void computeRN(const Eigen::Vector3d& r_BN_N, const Eigen::Vector3d& v_BN_N);
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> computeGuidanceSolution() const;

   private:
    FlybyPointConfig cfg;

    double dt = 0;               //!< [s] time step between last two updates — double: time arithmetic (§3.2 item 1)
    double timeOfFirstRead = 0;  //!< time of first nav solution read - double
    bool firstRead = true;       //!< variable to attest if this is the first read after a Reset
    float f0 = 0.0F;             //!< ratio between relative velocity and position norms at time of read [Hz]
    float gamma0 = 0.0F;         //!< flight path angle of the spacecraft at time of read [rad]
    uint64_t lastFilterReadTime = 0;                   //!< time of last filter read
    Eigen::Matrix3f R0N{Eigen::Matrix3f::Identity()};  //!< inertial-to-reference DCM at time of read
    Eigen::Vector3d firstNavPosition;                  //!< [m] position at first read — double
    Eigen::Vector3d firstNavVelocity;                  //!< [m/s] velocity at first read — double
};

#endif
