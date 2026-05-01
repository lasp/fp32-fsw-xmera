#ifndef F32XMERA_INERTIAL3D_H
#define F32XMERA_INERTIAL3D_H

#include "inertial3DAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <Eigen/Core>
#include <cstdint>

/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
class Inertial3D final : public SysModel {
   public:
    Inertial3D() = default;
    ~Inertial3D() override = default;

    void updateState(uint64_t callTime) override;
    void setSigmaRN(const Eigen::Vector3f& sigma_RN);
    Eigen::Vector3f getSigmaRN() const;

    Message<AttRefMsgF32Payload> attRefOutMsg;  //!< reference attitude output message

   private:
    Inertial3DAlgorithm algorithm{};
};

#endif
