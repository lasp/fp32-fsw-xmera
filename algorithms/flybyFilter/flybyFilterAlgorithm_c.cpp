#include "flybyFilterAlgorithm_c.h"

#include "flybyFilterAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

using FlybyFilterAlgorithm = ::filtering::flybyFilter::FlybyFilterAlgorithm;
using FlybyFilterConfig = ::filtering::flybyFilter::FlybyFilterConfig;
using FlybyFilterOutput = ::filtering::flybyFilter::FlybyFilterOutput;
using FlybyState = ::filtering::flybyFilter::FlybyState;
using HeadingData = ::filtering::flybyFilter::HeadingData;
using VehicleAttitude = ::filtering::flybyFilter::VehicleAttitude;
using StateMatrix = ::filtering::flybyFilter::StateMatrix;

namespace {

FlybyFilterConfig configFromC(const float alpha,
                              const float beta,
                              const float mu,
                              const FlybyFilterStateMatrix_c& processNoiseC,
                              const FlybyFilterStateVector_c& initialStateC,
                              const FlybyFilterStateMatrix_c& initialCovarianceC,
                              const float measurementNoiseScale,
                              const float maxAcrossTerminatorBias,
                              const float maxAlongTerminatorBias,
                              const float measurementDelay) {
    Eigen::Matrix<float, FLYBY_FILTER_NUM_STATES, 1> initialStateVec;
    for (int i = 0; i < FLYBY_FILTER_NUM_STATES; ++i) {
        initialStateVec(i) = initialStateC.data[i];
    }
    StateMatrix processNoise;
    StateMatrix initialCovariance;
    for (int i = 0; i < FLYBY_FILTER_NUM_STATES; ++i) {
        for (int j = 0; j < FLYBY_FILTER_NUM_STATES; ++j) {
            processNoise(i, j) = processNoiseC.data[(i * FLYBY_FILTER_NUM_STATES) + j];
            initialCovariance(i, j) = initialCovarianceC.data[(i * FLYBY_FILTER_NUM_STATES) + j];
        }
    }
    return FlybyFilterConfig::create(alpha,
                                     beta,
                                     mu,
                                     processNoise,
                                     FlybyState(initialStateVec),
                                     initialCovariance,
                                     measurementNoiseScale,
                                     maxAcrossTerminatorBias,
                                     maxAlongTerminatorBias,
                                     measurementDelay);
}

FlybyFilterOutput_c outputToC(const FlybyFilterOutput& out) {
    FlybyFilterOutput_c result{};
    for (int i = 0; i < FLYBY_FILTER_NUM_STATES; ++i) {
        result.state[i] = out.filterState.state(i);
        for (int j = 0; j < FLYBY_FILTER_NUM_STATES; ++j) {
            result.covariance[i][j] = out.filterState.covariance(i, j);
        }
    }
    result.headingResiduals.valid = out.headingResiduals.valid;
    for (int i = 0; i < 3; ++i) {
        result.headingResiduals.observation[i] = out.headingResiduals.observation(i);
        result.headingResiduals.preFit[i] = out.headingResiduals.preFit(i);
        result.headingResiduals.postFit[i] = out.headingResiduals.postFit(i);
    }
    return result;
}

}  // namespace

uint32_t FlybyFilterAlgorithm_getNumStates(void) { return FLYBY_FILTER_NUM_STATES; }

bool FlybyFilterAlgorithm_validateConfig(float alpha,
                                         float beta,
                                         float mu,
                                         const FlybyFilterStateMatrix_c* processNoise,
                                         const FlybyFilterStateVector_c* initialState,
                                         const FlybyFilterStateMatrix_c* initialCovariance,
                                         float measurementNoiseScale,
                                         float maxAcrossTerminatorBias,
                                         float maxAlongTerminatorBias,
                                         float measurementDelay) {
    // Build the config through the same path create() uses: success means valid, a throw means
    // invalid. Sharing configFromC keeps the predicate from drifting from what create() accepts.
    try {
        (void)configFromC(alpha,
                          beta,
                          mu,
                          *processNoise,
                          *initialState,
                          *initialCovariance,
                          measurementNoiseScale,
                          maxAcrossTerminatorBias,
                          maxAlongTerminatorBias,
                          measurementDelay);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

FlybyFilterAlgorithmHandle* FlybyFilterAlgorithm_create(float alpha,
                                                        float beta,
                                                        float mu,
                                                        const FlybyFilterStateMatrix_c* processNoise,
                                                        const FlybyFilterStateVector_c* initialState,
                                                        const FlybyFilterStateMatrix_c* initialCovariance,
                                                        float measurementNoiseScale,
                                                        float maxAcrossTerminatorBias,
                                                        float maxAlongTerminatorBias,
                                                        float measurementDelay) {
    return fsw::createHandle<FlybyFilterAlgorithm, FlybyFilterAlgorithmHandle>(configFromC(alpha,
                                                                                           beta,
                                                                                           mu,
                                                                                           *processNoise,
                                                                                           *initialState,
                                                                                           *initialCovariance,
                                                                                           measurementNoiseScale,
                                                                                           maxAcrossTerminatorBias,
                                                                                           maxAlongTerminatorBias,
                                                                                           measurementDelay));
}

void FlybyFilterAlgorithm_destroy(FlybyFilterAlgorithmHandle* self) { fsw::deleteHandle<FlybyFilterAlgorithm>(self); }

void FlybyFilterAlgorithm_setConfig(FlybyFilterAlgorithmHandle* self,
                                    float alpha,
                                    float beta,
                                    float mu,
                                    const FlybyFilterStateMatrix_c* processNoise,
                                    const FlybyFilterStateVector_c* initialState,
                                    const FlybyFilterStateMatrix_c* initialCovariance,
                                    float measurementNoiseScale,
                                    float maxAcrossTerminatorBias,
                                    float maxAlongTerminatorBias,
                                    float measurementDelay) {
    fsw::fromHandle<FlybyFilterAlgorithm>(self)->setConfig(configFromC(alpha,
                                                                       beta,
                                                                       mu,
                                                                       *processNoise,
                                                                       *initialState,
                                                                       *initialCovariance,
                                                                       measurementNoiseScale,
                                                                       maxAcrossTerminatorBias,
                                                                       maxAlongTerminatorBias,
                                                                       measurementDelay));
}

void FlybyFilterAlgorithm_reInitializeExceptPersistentStates(FlybyFilterAlgorithmHandle* self) {
    fsw::fromHandle<FlybyFilterAlgorithm>(self)->reInitializeExceptPersistentStates();
}

void FlybyFilterAlgorithm_reInitialize(FlybyFilterAlgorithmHandle* self) {
    fsw::fromHandle<FlybyFilterAlgorithm>(self)->reInitialize();
}

FlybyFilterOutput_c FlybyFilterAlgorithm_update(FlybyFilterAlgorithmHandle* self,
                                                double currentSeconds,
                                                const FlybyHeadingData_c* heading,
                                                const FlybyVehicleAttitude_c* attitude) {
    HeadingData in{};
    VehicleAttitude att{};
    in.timeTag = heading->timeTag;
    for (int i = 0; i < 3; ++i) {
        in.rhat_BN_N(i) = heading->rhat_BN_N[i];
        for (int j = 0; j < 3; ++j) {
            in.covar_N(i, j) = heading->covar_N[i][j];
        }
        att.sigma_BN(i) = attitude->sigma_BN[i];
        att.vehSunPntBdy(i) = attitude->vehSunPntBdy[i];
    }

    auto* algo = fsw::fromHandle<FlybyFilterAlgorithm>(self);
    const FlybyFilterOutput out = algo->update(currentSeconds, in, att);
    return outputToC(out);
}
