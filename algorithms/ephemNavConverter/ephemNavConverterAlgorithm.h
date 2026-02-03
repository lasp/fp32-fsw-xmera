#ifndef F32XMERA_EPHEM_NAV_CONVERTER_ALGORITHM_H
#define F32XMERA_EPHEM_NAV_CONVERTER_ALGORITHM_H

#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

/*! @brief The ephemNavConverter algorithm class.*/
class EphemNavConverterAlgorithm {
   public:
    EphemNavConverterAlgorithm() = default;   //!< Constructor
    ~EphemNavConverterAlgorithm() = default;  //!< Destructor

    static NavTransMsgF32Payload update(const EphemerisMsgF32Payload& ephemerisInMsg);
};

#endif
