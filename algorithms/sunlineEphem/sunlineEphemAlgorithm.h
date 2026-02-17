#ifndef F32XMERA_SUNLINE_EPHEM_ALGORITHM_H
#define F32XMERA_SUNLINE_EPHEM_ALGORITHM_H

#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

class SunlineEphemAlgorithm {
   public:
    NavAttMsgF32Payload updateState(const EphemerisMsgF32Payload& sunPos,
                                    const NavTransMsgF32Payload& scPos,
                                    const NavAttMsgF32Payload& scAtt) const;
};

#endif
