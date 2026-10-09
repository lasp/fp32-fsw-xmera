#include "dvManeuverAlgorithm.h"

DvManeuverAlgorithm::DvManeuverAlgorithm(const DvManeuverConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void DvManeuverAlgorithm::setConfig(const DvManeuverConfig& config) {
    this->cfg = config;
    this->cmdDvMagnitude = this->cfg.getCmdDv().stableNorm();
}

void DvManeuverAlgorithm::reInitialize() {
    this->state = DvManeuverBurnState::Executing;
    this->burnTime = 0U;
}

DvManeuverOutput DvManeuverAlgorithm::update(const Eigen::Vector3f& dvAccumulated) {
    if (this->state == DvManeuverBurnState::Executing) {
        const bool dvReached = dvAccumulated.stableNorm() >= this->cmdDvMagnitude;
        if ((dvReached && this->burnTime >= this->cfg.getMinTime()) || this->burnTime >= this->cfg.getMaxTime()) {
            this->state = DvManeuverBurnState::Complete;
        }
        this->burnTime += this->cfg.getControlPeriod();
    }

    // Complete is terminal: only reInitialize() leaves it, so reconfigure() can never reopen a finished burn
    DvManeuverOutput out{};
    out.state = this->state;
    if (this->state == DvManeuverBurnState::Executing) {
        out.cmdForce_B = this->cfg.getCmdForce();
    }
    return out;
}
