#ifndef XMERAF32_MRP_PD_ALGORITHM_H
#define XMERAF32_MRP_PD_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validInertiaCheck.h"
#include <stdint.h>
#include <Eigen/Core>

/*!
 * @brief Validated configuration for the MRP PD control algorithm.
 *
 * An instance can only exist with non-negative proportional and derivative gains and a valid spacecraft
 * inertia matrix. Construct via MrpPDConfig::create(...).
 */
class MrpPDConfig final {
   public:
    static MrpPDConfig create(float Kp, float Kd, const Eigen::Matrix3f& ISCPntB_B) {
        if (!isValidKp(Kp)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: proportional gain Kp must not be negative");
        }
        if (!isValidKd(Kd)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: derivative gain Kd must not be negative");
        }
        if (!isValidInertia(ISCPntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: ISCPntB_B must be a valid spacecraft inertia matrix");
        }
        return {Kp, Kd, ISCPntB_B};
    }

    static bool isValidKp(float Kp) { return Kp >= 0.0F; }
    static bool isValidKd(float Kd) { return Kd >= 0.0F; }
    static bool isValidInertia(const Eigen::Matrix3f& ISCPntB_B) { return inertiaIsValid(ISCPntB_B); }

    float getKp() const { return Kp; }
    float getKd() const { return Kd; }
    const Eigen::Matrix3f& getInertia() const { return ISCPntB_B; }

   private:
    // The two gains share a type but have distinct roles; construction is funneled through the named create()
    // factory, which makes the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    MrpPDConfig(float Kp, float Kd, const Eigen::Matrix3f& ISCPntB_B) : Kp(Kp), Kd(Kd), ISCPntB_B(ISCPntB_B) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    float Kp;
    float Kd;
    Eigen::Matrix3f ISCPntB_B;
};

/*! @brief MRP PD control algorithm class. */
class MrpPDAlgorithm final {
   public:
    explicit MrpPDAlgorithm(const MrpPDConfig& config);
    void setConfig(const MrpPDConfig& config);

    Eigen::Vector3f update(const Eigen::Vector3f& sigma_BR,
                           const Eigen::Vector3f& omega_BR_B,
                           const Eigen::Vector3f& domega_RN_B) const;

   private:
    MrpPDConfig cfg;
};

#endif
