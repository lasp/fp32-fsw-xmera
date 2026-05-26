#ifndef F32XMERA_MRP_FEEDBACK_TYPES_H
#define F32XMERA_MRP_FEEDBACK_TYPES_H

#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"

enum class ControlLawType { NORMAL = 0, SIMPLE_INTEGRAL = 1 };

struct MrpFeedbackOutput {
    CmdTorqueBodyMsgF32Payload controlOut{};      //!< control torque output
    CmdTorqueBodyMsgF32Payload intFeedbackOut{};  //!< integral feedback torque output
};

#endif
