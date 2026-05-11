#ifndef F32XMERA_DV_GUIDANCE_TYPES_H
#define F32XMERA_DV_GUIDANCE_TYPES_H

#include <Eigen/Core>

struct DvGuidanceOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
};

// dvGuidance has no tunable parameters; the Config class is intentionally empty so the
// algorithm can still follow the standard two-phase init pattern.
class DvGuidanceConfig final {
   public:
    static DvGuidanceConfig create() { return {}; }

   private:
    DvGuidanceConfig() = default;
};

#endif
