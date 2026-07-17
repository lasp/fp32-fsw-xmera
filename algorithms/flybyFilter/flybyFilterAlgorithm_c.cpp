#include "flybyFilterAlgorithm_c.h"

#include "flybyFilterAlgorithm.h"

#include <Eigen/Core>

using FlybyFilterAlgorithm = ::filtering::flybyFilter::FlybyFilterAlgorithm;
using FlybyFilterConfig = ::filtering::flybyFilter::FlybyFilterConfig;
using FlybyFilterOutput = ::filtering::flybyFilter::FlybyFilterOutput;
using FlybyState = ::filtering::flybyFilter::FlybyState;
using HeadingData = ::filtering::flybyFilter::HeadingData;
using StateMatrix = ::filtering::flybyFilter::StateMatrix;

namespace {

FlybyFilterConfig configFromC(const FlybyFilterConfig_c& c) {
    Eigen::Matrix<double, FLYBY_FILTER_NUM_STATES, 1> initialStateVec;
    for (int i = 0; i < FLYBY_FILTER_NUM_STATES; ++i) {
        initialStateVec(i) = c.initialState[i];
    }
    StateMatrix processNoise;
    StateMatrix initialCovariance;
    for (int i = 0; i < FLYBY_FILTER_NUM_STATES; ++i) {
        for (int j = 0; j < FLYBY_FILTER_NUM_STATES; ++j) {
            processNoise(i, j) = c.processNoise[i][j];
            initialCovariance(i, j) = c.initialCovariance[i][j];
        }
    }
    return FlybyFilterConfig::create(c.alpha,
                                     c.beta,
                                     c.mu,
                                     processNoise,
                                     FlybyState(initialStateVec),
                                     initialCovariance,
                                     c.headingMeasurementNoiseStd);
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

FlybyFilterAlgorithmHandle* FlybyFilterAlgorithm_create(const FlybyFilterConfig_c* config) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    return reinterpret_cast<FlybyFilterAlgorithmHandle*>(new FlybyFilterAlgorithm(configFromC(*config)));
}

void FlybyFilterAlgorithm_destroy(FlybyFilterAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    delete reinterpret_cast<FlybyFilterAlgorithm*>(self);
}

void FlybyFilterAlgorithm_reInitializeExceptPersistentStates(FlybyFilterAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<FlybyFilterAlgorithm*>(self)->reInitializeExceptPersistentStates();
}

void FlybyFilterAlgorithm_reInitialize(FlybyFilterAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<FlybyFilterAlgorithm*>(self)->reInitialize();
}

FlybyFilterOutput_c FlybyFilterAlgorithm_update(FlybyFilterAlgorithmHandle* self,
                                                double currentSeconds,
                                                const FlybyHeadingData_c* heading) {
    HeadingData in{};
    in.timeTag = heading->timeTag;
    in.rhat_BN_N = Eigen::Vector3d(heading->rhat_BN_N[0], heading->rhat_BN_N[1], heading->rhat_BN_N[2]);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* algo = reinterpret_cast<FlybyFilterAlgorithm*>(self);
    const FlybyFilterOutput out = algo->update(currentSeconds, in);
    return outputToC(out);
}
