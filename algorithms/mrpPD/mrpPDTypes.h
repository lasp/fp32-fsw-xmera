// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERAF32_MRP_PD_TYPES_H
#define XMERAF32_MRP_PD_TYPES_H

#include "utilities/freestandingInvalidArgument.h"
#include "utilities/validInertiaCheck.h"
#include <Eigen/Core>

class MrpPDConfig final {
   public:
    static MrpPDConfig create(float proportionalGainK,
                              float derivativeGainP,
                              const Eigen::Vector3f& knownTorquePntB_B,
                              const Eigen::Matrix3f& spacecraftInertia) {
        if (!isValidProportionalGainK(proportionalGainK)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: proportionalGainK must be non-negative");
        }
        if (!isValidDerivativeGainP(derivativeGainP)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: derivativeGainP must be non-negative");
        }
        if (!isValidKnownTorquePntB_B(knownTorquePntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: knownTorquePntB_B must be finite");
        }
        if (!isValidSpacecraftInertia(spacecraftInertia)) {
            FSW_THROW_INVALID_ARGUMENT("mrpPD: spacecraftInertia must pass inertia validity checks");
        }
        return {proportionalGainK, derivativeGainP, knownTorquePntB_B, spacecraftInertia};
    }

    static bool isValidProportionalGainK(float proportionalGainK) { return proportionalGainK >= 0.0F; }
    static bool isValidDerivativeGainP(float derivativeGainP) { return derivativeGainP >= 0.0F; }
    static bool isValidKnownTorquePntB_B(const Eigen::Vector3f& knownTorquePntB_B) {
        return knownTorquePntB_B.allFinite();
    }
    static bool isValidSpacecraftInertia(const Eigen::Matrix3f& spacecraftInertia) {
        return inertiaIsValid(spacecraftInertia);
    }

    float getProportionalGainK() const { return this->proportionalGainK; }
    float getDerivativeGainP() const { return this->derivativeGainP; }
    const Eigen::Vector3f& getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
    const Eigen::Matrix3f& getSpacecraftInertia() const { return this->spacecraftInertia; }

   private:
    MrpPDConfig(float proportionalGainK,
                float derivativeGainP,
                const Eigen::Vector3f& knownTorquePntB_B,
                const Eigen::Matrix3f& spacecraftInertia)
        : proportionalGainK(proportionalGainK),
          derivativeGainP(derivativeGainP),
          knownTorquePntB_B(knownTorquePntB_B),
          spacecraftInertia(spacecraftInertia) {}

    float proportionalGainK;
    float derivativeGainP;
    Eigen::Vector3f knownTorquePntB_B;
    Eigen::Matrix3f spacecraftInertia;
};

#endif
