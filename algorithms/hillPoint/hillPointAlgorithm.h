#ifndef F32XMERA_HILL_POINT_ALGORITHM_H
#define F32XMERA_HILL_POINT_ALGORITHM_H

#include <Eigen/Core>
#include "hillPointTypes.h"

struct HillPointOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();
};

// hillPoint has no tunable parameters; the Config class is intentionally empty so the
// algorithm can still follow the standard two-phase init pattern.
class HillPointConfig final {
   public:
    static HillPointConfig create() { return {}; }

   private:
    HillPointConfig() = default;
};

class HillPointAlgorithm final {
   public:
    explicit HillPointAlgorithm(const HillPointConfig& config);

    void setConfig(const HillPointConfig& config);

    HillPointOutput update(const Eigen::Vector3d& r_BN_N,
                           const Eigen::Vector3d& v_BN_N,
                           const Eigen::Vector3d& r_planet_N,
                           const Eigen::Vector3d& v_planet_N) const;

   private:
    HillPointConfig cfg;
};

#endif
