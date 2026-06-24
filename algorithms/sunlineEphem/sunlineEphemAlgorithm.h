#ifndef F32XMERA_SUNLINE_EPHEM_ALGORITHM_H
#define F32XMERA_SUNLINE_EPHEM_ALGORITHM_H

#include <Eigen/Core>

// sunlineEphem has no tunable parameters; the Config class is intentionally empty so the
// algorithm can still follow the standard two-phase init pattern.
class SunlineEphemConfig final {
   public:
    static SunlineEphemConfig create() { return {}; }

   private:
    SunlineEphemConfig() = default;
};

class SunlineEphemAlgorithm final {
   public:
    explicit SunlineEphemAlgorithm(const SunlineEphemConfig& config);

    void setConfig(const SunlineEphemConfig& config);

    Eigen::Vector3f update(const Eigen::Vector3d& r_SN_N,
                           const Eigen::Vector3d& r_BN_N,
                           const Eigen::Vector3f& sigma_BN) const;

   private:
    SunlineEphemConfig cfg;
};

#endif
