#include "dvManeuverAlgorithm.h"

DvManeuverAlgorithm::DvManeuverAlgorithm(const DvManeuverConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void DvManeuverAlgorithm::setConfig(const DvManeuverConfig& config) {
    this->cfg = config;
    this->cmdDvMagnitude = this->cfg.getCmdDv().stableNorm();
    this->minTimeNs = dvManeuverToNanoseconds(this->cfg.getMinTime());
    this->maxTimeNs = dvManeuverToNanoseconds(this->cfg.getMaxTime());
    this->controlPeriodNs = dvManeuverToNanoseconds(this->cfg.getControlPeriod());
}

void DvManeuverAlgorithm::reInitialize() {
    this->state = DvManeuverBurnState::Executing;
    this->burnTime = 0U;
}

DvManeuverOutput DvManeuverAlgorithm::update(const Eigen::Vector3f& dvAccumulated_B) {
    if (this->state == DvManeuverBurnState::Executing) {
        const bool dvReached = dvAccumulated_B.stableNorm() >= this->cmdDvMagnitude;
        if ((dvReached && this->burnTime >= this->minTimeNs) || this->burnTime >= this->maxTimeNs) {
            this->state = DvManeuverBurnState::Complete;
        }
        this->burnTime += this->controlPeriodNs;
    }

    // Complete is terminal: only reInitialize() leaves it, so reconfigure() can never reopen a finished burn
    DvManeuverOutput out{};
    out.state = this->state;
    if (this->state == DvManeuverBurnState::Executing) {
        out.cmdForce_B = this->cfg.getCmdForce();
    }
    return out;
}
