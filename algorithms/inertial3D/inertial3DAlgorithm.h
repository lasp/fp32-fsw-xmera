#ifndef F32XMERA_INERTIAL3D_ALGORITHM_H
#define F32XMERA_INERTIAL3D_ALGORITHM_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <Eigen/Core>

/*!
 * @brief Validated configuration for the inertial-3D pointing algorithm.
 *
 * An instance can only exist with a finite reference-attitude MRP. Construct via Inertial3DConfig::create(...).
 */
class Inertial3DConfig final {
   public:
    static Inertial3DConfig create(const Eigen::Vector3f& sigma_RN) {
        if (!isValidSigmaRN(sigma_RN)) {
            FSW_THROW_INVALID_ARGUMENT("inertial3D: sigma_RN must be finite");
        }
        return Inertial3DConfig{sigma_RN};
    }

    static bool isValidSigmaRN(const Eigen::Vector3f& sigma_RN) { return sigma_RN.allFinite(); }

    const Eigen::Vector3f& getSigmaRN() const { return sigma_RN; }

   private:
    explicit Inertial3DConfig(const Eigen::Vector3f& sigma_RN) : sigma_RN(sigma_RN) {}

    Eigen::Vector3f sigma_RN;
};

/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
class Inertial3DAlgorithm final {
   public:
    AttRefMsgF32Payload update() const;
    void setSigmaRN(const Eigen::Vector3f& sigmaInput_RN);
    Eigen::Vector3f getSigmaRN() const;

   private:
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};  //!<  MRP from inertial frame N to reference frame R
};

#endif
