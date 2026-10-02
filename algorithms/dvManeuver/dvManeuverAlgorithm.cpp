#include "dvManeuverAlgorithm.h"
#include "utilities/fsw/timeConstants.h"

DvManeuverAlgorithm::DvManeuverAlgorithm(const DvManeuverConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

void DvManeuverAlgorithm::setConfig(const DvManeuverConfig& config) { this->cfg = config; }

void DvManeuverAlgorithm::reInitialize() {
    this->state = DvManeuverBurnState::Pending;
    this->burnTime = 0.0F;
    this->dvInitial = Eigen::Vector3f::Zero();
}

DvManeuverOutput DvManeuverAlgorithm::update(const uint64_t callTime, const Eigen::Vector3f& dvAccumulated) {
    if (this->state == DvManeuverBurnState::Pending && callTime >= this->cfg.getBurnStartTime()) {
        this->state = DvManeuverBurnState::Executing;
        this->dvInitial = dvAccumulated;
    }

    if (this->state == DvManeuverBurnState::Executing) {
        this->burnTime += this->cfg.getControlPeriod();
        const bool dvReached = (dvAccumulated - this->dvInitial).stableNorm() >= this->cfg.getCmdDv().stableNorm();
        const float minTime = static_cast<float>(this->cfg.getMinTime()) / kSec2NanoF;
        const float maxTime = static_cast<float>(this->cfg.getMaxTime()) / kSec2NanoF;
        if ((dvReached && this->burnTime > minTime) || this->burnTime > maxTime) {
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
