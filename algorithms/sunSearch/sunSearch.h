/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XIMERA_SUN_SEARCH_H
#define F32XIMERA_SUN_SEARCH_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "sunSearchAlgorithm.h"

class SunSearch : public SysModel {
   public:
    SunSearch() = default;
    ~SunSearch() = default;

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void setSlewProperties(const SlewProperties& slewPropertiesInput);
    void modifySlewProperties(const SlewProperties& slewPropertiesInput, uint32_t index);
    SlewProperties getSlewProperties(uint32_t index) const;

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;            //!< input msg measured attitude
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< input veh config msg
    Message<AttGuidMsgF32Payload> attGuidOutMsg;             //!< Attitude reference output message

   private:
    SunSearchAlgorithm algorithm{};
};

#endif
