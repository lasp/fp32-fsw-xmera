/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

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
    void setSlewProperties(SlewProperties slewPropertiesInput);
    void modifySlewProperties(SlewProperties slewPropertiesInput, uint32_t index);
    SlewProperties getSlewProperties(uint32_t index) const;

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;            //!< input msg measured attitude
    ReadFunctor<VehicleConfigMsgF32Payload> vehConfigInMsg;  //!< input veh config msg
    Message<AttGuidMsgF32Payload> attGuidOutMsg;             //!< Attitude reference output message

   private:
    SunSearchAlgorithm algorithm{};
};

#endif
