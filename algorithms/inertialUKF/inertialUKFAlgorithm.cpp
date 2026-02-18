/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "inertialUKFAlgorithm.h"

InertialUKFOutput InertialUKFAlgorithm::updateState(const STAttInput& stAtt,
                                                    const GyroInput& gyro,
                                                    const RWSpeedsInput& rwSpeeds,
                                                    const RWArrayConfigInput& rwConfig,
                                                    const VehicleConfigInput& vehConfig) {
    InertialUKFOutput output{};

    // Pass through the star tracker attitude as the nav attitude estimate
    output.navAtt.timeTag = stAtt.timeTag;
    output.navAtt.sigma_BN = stAtt.MRP_BdyInrtl;
    output.navAtt.omega_BN_B = stAtt.omega_BN_B;

    output.filter.timeTag = stAtt.timeTag;

    return output;
}
