/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XMERA_MRP_STEERING_ALGORITHM_H
#define F32XMERA_MRP_STEERING_ALGORITHM_H

#include "freestandingInvalidArgument.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/RateCmdMsgF32Payload.h"

class MrpSteeringConfig final {
   public:
    static /*std::optional*/ MrpSteeringConfig create(const float k1, const float k3, const float omegaMax, const bool ignoreOuterLoopFeedforward) {
        if (!isValidK1(k1)) {
            FS_THROW_INVALID_ARGUMENT("mrpSteering feedback gain k1 must not be negative");
        }
        if (!isValidK3(k3)) {
            FS_THROW_INVALID_ARGUMENT("mrpSteering feedback gain k3 must not be negative");
        }
        if (!isValidOmegaMax(omegaMax)) {
            FS_THROW_INVALID_ARGUMENT("mrpSteering maximum rate omegaMax must be positive");
        }
        return {k1, k3, omegaMax, ignoreOuterLoopFeedforward};
    }

    static bool isValidK1(const float k1) { return k1 >= 0.0F; }
    static bool isValidK3(const float k3) { return k3 >= 0.0F; }
    static bool isValidOmegaMax(const float omegaMax) { return omegaMax > 0.0F; }

    float getK1() const { return k1; }
    float getK3() const { return k3; }
    float getOmegaMax() const { return omegaMax; }
    bool getIgnoreOuterLoopFeedforward() const { return ignoreOuterLoopFeedforward; }

    private:
        MrpSteeringConfig(const float k1, const float k3, const float omegaMax, const bool ignoreOuterLoopFeedforward)
        : k1(k1), k3(k3), omegaMax(omegaMax), ignoreOuterLoopFeedforward(ignoreOuterLoopFeedforward) {}

        float k1;
        float k3;
        float omegaMax;
        bool ignoreOuterLoopFeedforward;
};

/*! @brief Data structure for the MRP feedback attitude control routine. */
class MrpSteeringAlgorithm final {
   public:
    explicit MrpSteeringAlgorithm(const MrpSteeringConfig& config);

    void setConfig(const MrpSteeringConfig& config);

    RateCmdMsgF32Payload update(AttGuidMsgF32Payload& guidInMsg) const;

   private:
    MrpSteeringConfig cfg;
};

#endif
