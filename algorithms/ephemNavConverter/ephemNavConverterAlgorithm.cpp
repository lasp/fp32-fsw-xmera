/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "ephemNavConverterAlgorithm.h"

static void v3Copy(float const v[3], float result[3]) {
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
}

/*! Update method for the ephemNavConverter algorithm. This method reads in the ephemeris messages and copies the
 translation ephemeris to the navigation translation interface message.
 @return NavTransMsgF32Payload Translational navigation message
 @param callTime [ns] Time the method is called
 @param ephemerisInMsg Ephemeris message
 */
NavTransMsgF32Payload
EphemNavConverterAlgorithm::update(uint64_t callTime, const EphemerisMsgF32Payload &ephemerisInMsg) const {
    // Create the output message
    auto navTransMsgBuffer = NavTransMsgF32Payload{};

    // Map timeTag, position and velocity vector to output message
    navTransMsgBuffer.timeTag = ephemerisInMsg.timeTag;
    v3Copy(ephemerisInMsg.r_BdyZero_N, navTransMsgBuffer.r_BN_N);
    v3Copy(ephemerisInMsg.v_BdyZero_N, navTransMsgBuffer.v_BN_N);

    return navTransMsgBuffer;
}
