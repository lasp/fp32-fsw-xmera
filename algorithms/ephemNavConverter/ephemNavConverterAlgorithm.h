#ifndef F32XMERA_EPHEM_NAV_CONVERTER_ALGORITHM_H
#define F32XMERA_EPHEM_NAV_CONVERTER_ALGORITHM_H

#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <stdint.h>

/*! @brief The ephemNavConverter algorithm class.*/
class EphemNavConverterAlgorithm {
   public:
    EphemNavConverterAlgorithm() = default;   //!< Constructor
    ~EphemNavConverterAlgorithm() = default;  //!< Destructor

    NavTransMsgF32Payload update(uint64_t callTime, const EphemerisMsgF32Payload& ephemerisInMsg) const;
};

#endif
