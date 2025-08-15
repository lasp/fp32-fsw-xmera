/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "sunlineEphem.h"

void SunlineEphem::reset(uint64_t callTime) {
    assert(this->sunPositionInMsg.isLinked());
    assert(this->scPositionInMsg.isLinked());
    assert(this->scAttitudeInMsg.isLinked());
}

void SunlineEphem::updateState(uint64_t callTime) {
    EphemerisMsgF32Payload sunPos = this->sunPositionInMsg();
    NavTransMsgF32Payload scPos = this->scPositionInMsg();
    NavAttMsgF32Payload scAtt = this->scAttitudeInMsg();
    auto outputSunline = this->algorithm.updateState(sunPos, scPos, scAtt);
    this->navStateOutMsg.write(&outputSunline, this->moduleID, callTime);
}
