#include "ephemNavConverterAlgorithm.h"

/*! Update method for the ephemNavConverter algorithm. This method reads in the ephemeris messages and copies the
 translation ephemeris to the navigation translation interface message.
 @return OutputNavTransData Translational navigation output
 @param ephemerisInput Ephemeris input
 */
OutputNavTransData EphemNavConverterAlgorithm::update(const InputEphemerisData& ephemerisInput) {
    OutputNavTransData navTransOutput{};

    // Map timeTag, position and velocity vector to translational navigation output struct
    navTransOutput.timeTag = ephemerisInput.timeTag;
    navTransOutput.r_BN_N = ephemerisInput.r_BdyZero_N;
    navTransOutput.v_BN_N = ephemerisInput.v_BdyZero_N;

    return navTransOutput;
}
