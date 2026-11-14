#include "dvManeuverAlgorithm.h"

DvManeuverAlgorithm::DvManeuverAlgorithm(const DvManeuverConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void DvManeuverAlgorithm::setConfig(const DvManeuverConfig& config) {
    this->cfg = config;
    this->requestedDvMagnitude = this->cfg.getRequestedDv().stableNorm();
    // DvManeuverConfig::create() makes sure that each time converts. Thus the code never uses the fallbacks.
    this->minTimeNs = fsw::secondsToNanos(this->cfg.getMinTime()).value_or(0U);
    this->maxTimeNs = fsw::secondsToNanos(this->cfg.getMaxTime()).value_or(0U);
    this->controlPeriodNs = fsw::secondsToNanos(this->cfg.getControlPeriod()).value_or(0U);
}

void DvManeuverAlgorithm::reInitialize() {
    this->state = DvManeuverBurnState::Executing;
    this->burnTime = 0U;
}

DvManeuverOutput DvManeuverAlgorithm::update(const Eigen::Vector3f& dvAccumulated_B) {
    if (this->state == DvManeuverBurnState::Executing) {
        const bool dvReached = dvAccumulated_B.stableNorm() >= this->requestedDvMagnitude;
        if ((dvReached && this->burnTime >= this->minTimeNs) || this->burnTime >= this->maxTimeNs) {
            this->state = DvManeuverBurnState::Complete;
        }
        this->burnTime += this->controlPeriodNs;
    }

    // Complete is terminal: only reInitialize() leaves it, so reconfigure() can never reopen a finished burn
    DvManeuverOutput out{};
    out.state = this->state;
    if (this->state == DvManeuverBurnState::Executing) {
        out.requestedForce_B = this->cfg.getRequestedForce();
    }
    return out;
}
