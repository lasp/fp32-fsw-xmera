#ifndef F32XMERA_MRP_ROTATION_TYPES_H
#define F32XMERA_MRP_ROTATION_TYPES_H

#include "utilities/freestandingInvalidArgument.h"
#include <Eigen/Core>

class MrpRotationConfig final {
   public:
    static MrpRotationConfig create(const Eigen::Vector3f& initialSigmaRR0,
                                    const Eigen::Vector3f& omegaRR0R,
                                    bool dynamicReferenceEnabled) {
        if (!isValidInitialSigmaRR0(initialSigmaRR0)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: initialSigmaRR0 must be finite");
        }
        if (!isValidOmegaRR0R(omegaRR0R)) {
            FSW_THROW_INVALID_ARGUMENT("mrpRotation: omegaRR0R must be finite");
        }
        return {initialSigmaRR0, omegaRR0R, dynamicReferenceEnabled};
    }

    static bool isValidInitialSigmaRR0(const Eigen::Vector3f& sigma) { return sigma.allFinite(); }
    static bool isValidOmegaRR0R(const Eigen::Vector3f& omega) { return omega.allFinite(); }

    Eigen::Vector3f getInitialSigmaRR0() const { return initialSigmaRR0; }
    Eigen::Vector3f getOmegaRR0R() const { return omegaRR0R; }
    bool getDynamicReferenceEnabled() const { return dynamicReferenceEnabled; }

   private:
    MrpRotationConfig(const Eigen::Vector3f& initialSigmaRR0,
                      const Eigen::Vector3f& omegaRR0R,
                      bool dynamicReferenceEnabled)
        : initialSigmaRR0(initialSigmaRR0), omegaRR0R(omegaRR0R), dynamicReferenceEnabled(dynamicReferenceEnabled) {}

    Eigen::Vector3f initialSigmaRR0;
    Eigen::Vector3f omegaRR0R;
    bool dynamicReferenceEnabled;
};

#endif
