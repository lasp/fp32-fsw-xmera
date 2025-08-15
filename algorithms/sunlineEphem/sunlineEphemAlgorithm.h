/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XIMERA_SUNLINE_EPHEM_ALGORITHM_H
#define F32XIMERA_SUNLINE_EPHEM_ALGORITHM_H

#include <assert.h>
#include <stdint.h>
#include <Eigen/Core>

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

class SunlineEphemAlgorithm {
   public:
    NavAttMsgF32Payload updateState(const EphemerisMsgF32Payload &sunPos,
                                    const NavTransMsgF32Payload &scPos,
                                    const NavAttMsgF32Payload &scAtt) const;
};

#endif
