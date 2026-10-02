#include "dvManeuverAlgorithm.h"

DvManeuverAlgorithm::DvManeuverAlgorithm(const DvManeuverConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void DvManeuverAlgorithm::setConfig(const DvManeuverConfig& config) { this->cfg = config; }

void DvManeuverAlgorithm::reInitialize() {
    this->state = DvManeuverBurnState::Pending;
    this->burnTime = 0.0F;
    this->dvInit = Eigen::Vector3f::Zero();
}

DvManeuverOutput DvManeuverAlgorithm::update(const uint64_t callTime, const Eigen::Vector3f& vehAccumDV) {
    if (this->state == DvManeuverBurnState::Pending && callTime >= this->cfg.getBurnStartTime()) {
        this->state = DvManeuverBurnState::Executing;
        this->dvInit = vehAccumDV;
    }

    if (this->state == DvManeuverBurnState::Executing) {
        this->burnTime += this->cfg.getControlPeriod();
        const bool dvReached = (vehAccumDV - this->dvInit).stableNorm() >= this->cfg.getCmdDv().stableNorm();
        if ((dvReached && this->burnTime > this->cfg.getMinTime()) || this->burnTime > this->cfg.getMaxTime()) {
            this->state = DvManeuverBurnState::Complete;
        }
    }

    // Complete is terminal: only reInitialize() leaves it, so reconfigure() can never reopen a finished burn
    DvManeuverOutput out;
    out.state = this->state;
    if (this->state == DvManeuverBurnState::Executing) {
        out.cmdForce_B = this->cfg.getCmdForce();
    }
    return out;
}
