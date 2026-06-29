#ifndef XMERAF32_MRP_PD_ALGORITHM_H
#define XMERAF32_MRP_PD_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/validInertiaCheck.h"
#include <stdint.h>
#include <Eigen/Core>

/*!
 * @brief Validated configuration for the MRP PD control algorithm.
 *
 * An instance can only exist with non-negative proportional and derivative gains, a finite known external
 * torque, and a valid spacecraft inertia matrix. Construct via MrpPDConfig::create(...).
 */
class MrpPDConfig final {
   public:
    static MrpPDConfig create(float proportionalGainK,
                              float derivativeGainP,
                              const Eigen::Vector3f& knownTorquePntB_B,
                              const Eigen::Matrix3f& ISCPntB_B) {
        if (!isValidProportionalGainK(proportionalGainK)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: proportional gain K must not be negative");
        }
        if (!isValidDerivativeGainP(derivativeGainP)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: derivative gain P must not be negative");
        }
        if (!isValidKnownTorquePntB_B(knownTorquePntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: knownTorquePntB_B must be finite");
        }
        if (!isValidInertia(ISCPntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: ISCPntB_B must be a valid spacecraft inertia matrix");
        }
        return {proportionalGainK, derivativeGainP, knownTorquePntB_B, ISCPntB_B};
    }

    static bool isValidProportionalGainK(float proportionalGainK) { return proportionalGainK >= 0.0F; }
    static bool isValidDerivativeGainP(float derivativeGainP) { return derivativeGainP >= 0.0F; }
    static bool isValidKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B) {
        return knownTorquePntB_B.allFinite();
    }
    static bool isValidInertia(const Eigen::Matrix3f& ISCPntB_B) { return inertiaIsValid(ISCPntB_B); }

    float getProportionalGainK() const { return proportionalGainK; }
    float getDerivativeGainP() const { return derivativeGainP; }
    const Eigen::Vector3f& getKnownTorquePntB_B() const { return knownTorquePntB_B; }
    const Eigen::Matrix3f& getInertia() const { return ISCPntB_B; }

   private:
    // The two gains share a type but have distinct roles; construction is funneled through the named create()
    // factory, which makes the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    MrpPDConfig(float proportionalGainK,
                float derivativeGainP,
                const Eigen::Vector3f& knownTorquePntB_B,
                const Eigen::Matrix3f& ISCPntB_B)
        : proportionalGainK(proportionalGainK),
          derivativeGainP(derivativeGainP),
          knownTorquePntB_B(knownTorquePntB_B),
          ISCPntB_B(ISCPntB_B) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    float proportionalGainK;
    float derivativeGainP;
    Eigen::Vector3f knownTorquePntB_B;
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
