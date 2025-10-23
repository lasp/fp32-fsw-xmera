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

#ifndef F32XIMERA_CELESTIAL_BODY_POINT_H
#define F32XIMERA_CELESTIAL_BODY_POINT_H

#include <stdint.h>
#include <stdexcept>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "celestialTwoBodyPointAlgorithm.h"

/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
class CelestialTwoBodyPoint : public SysModel {
   public:
    CelestialTwoBodyPoint() = default;
    ~CelestialTwoBodyPoint() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setSingularityThresh(float thresh);
    float getSingularityThresh() const;

    Message<AttRefMsgF32Payload> attRefOutMsg;            //!< The name of the output message*/
    ReadFunctor<EphemerisMsgF32Payload> celBodyInMsg;     //!< The name of the celestial body message*/
    ReadFunctor<EphemerisMsgF32Payload> secCelBodyInMsg;  //!< The name of the secondary body to constrain point*/
    ReadFunctor<NavTransMsgF32Payload> transNavInMsg;     //!< The name of the incoming attitude command*/

   private:
    bool secCelBodyIsLinked{};
    CelestialTwoBodyPointAlgorithm algorithm{};
};

#endif
