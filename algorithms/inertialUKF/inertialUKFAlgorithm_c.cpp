/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics,
 University of Colorado at Boulder
 */

#include "inertialUKFAlgorithm_c.h"
#include "inertialUKFAlgorithm.h"

#include <Eigen/Core>

InertialUKFOutput_c InertialUKFAlgorithm_updateState(const STAttInput_c* stAtt,
                                                     const GyroInput_c* gyro,
                                                     const RWSpeedsInput_c* rwSpeeds,
                                                     const RWArrayConfigInput_c* rwConfig,
                                                     const VehicleConfigInput_c* vehConfig) {
    // Convert C POD inputs to Eigen-based algorithm structs
    STAttInput stAttCpp{};
    stAttCpp.timeTag = stAtt->timeTag;
    stAttCpp.MRP_BdyInrtl << stAtt->MRP_BdyInrtl[0], stAtt->MRP_BdyInrtl[1], stAtt->MRP_BdyInrtl[2];
    stAttCpp.omega_BN_B << stAtt->omega_BN_B[0], stAtt->omega_BN_B[1], stAtt->omega_BN_B[2];

    GyroInput gyroCpp{};
    gyroCpp.gyro_B << gyro->gyro_B[0], gyro->gyro_B[1], gyro->gyro_B[2];

    RWSpeedsInput rwSpeedsCpp{};
    rwSpeedsCpp.wheelSpeeds << rwSpeeds->wheelSpeeds[0], rwSpeeds->wheelSpeeds[1], rwSpeeds->wheelSpeeds[2],
        rwSpeeds->wheelSpeeds[3];

    RWArrayConfigInput rwConfigCpp{};
    rwConfigCpp.numRW = rwConfig->numRW;

    VehicleConfigInput vehConfigCpp{};
    vehConfigCpp.ISCPntB_B = Eigen::Map<const Eigen::Matrix3f>(vehConfig->ISCPntB_B);
    vehConfigCpp.massSC = vehConfig->massSC;

    // Call the static algorithm method
    const InertialUKFOutput output =
        InertialUKFAlgorithm::updateState(stAttCpp, gyroCpp, rwSpeedsCpp, rwConfigCpp, vehConfigCpp);

    // Convert output back to C POD types
    InertialUKFOutput_c out{};
    out.navAtt.timeTag = output.navAtt.timeTag;
    out.navAtt.sigma_BN[0] = output.navAtt.sigma_BN[0];
    out.navAtt.sigma_BN[1] = output.navAtt.sigma_BN[1];
    out.navAtt.sigma_BN[2] = output.navAtt.sigma_BN[2];
    out.navAtt.omega_BN_B[0] = output.navAtt.omega_BN_B[0];
    out.navAtt.omega_BN_B[1] = output.navAtt.omega_BN_B[1];
    out.navAtt.omega_BN_B[2] = output.navAtt.omega_BN_B[2];
    out.navAtt.vehSunPntBdy[0] = output.navAtt.vehSunPntBdy[0];
    out.navAtt.vehSunPntBdy[1] = output.navAtt.vehSunPntBdy[1];
    out.navAtt.vehSunPntBdy[2] = output.navAtt.vehSunPntBdy[2];
    out.filter.timeTag = output.filter.timeTag;
    out.filter.numObs = output.filter.numObs;

    return out;
}
