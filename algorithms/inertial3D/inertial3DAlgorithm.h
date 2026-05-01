#ifndef F32XMERA_INERTIAL3D_ALGORITHM_H
#define F32XMERA_INERTIAL3D_ALGORITHM_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include <Eigen/Core>

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
