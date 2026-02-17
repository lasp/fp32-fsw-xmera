#include "sunlineEphem.h"

#include <stdexcept>

void SunlineEphem::reset(uint64_t callTime) {
    if (!this->sunPositionInMsg.isLinked()) {
        throw std::invalid_argument("SunlineEphem.sunPositionInMsg is unlinked");
    }
    if (!this->scPositionInMsg.isLinked()) {
        throw std::invalid_argument("SunlineEphem.scPositionInMsg is unlinked");
    }
    if (!this->scAttitudeInMsg.isLinked()) {
        throw std::invalid_argument("SunlineEphem.scAttitudeInMsg is unlinked");
    }
}

void SunlineEphem::updateState(uint64_t callTime) {
    const EphemerisMsgF32Payload sunPos = this->sunPositionInMsg();
    const NavTransMsgF32Payload scPos = this->scPositionInMsg();
    const NavAttMsgF32Payload scAtt = this->scAttitudeInMsg();
    auto outputSunline = this->algorithm.updateState(sunPos, scPos, scAtt);
    this->navStateOutMsg.write(&outputSunline, this->moduleID, callTime);
}
