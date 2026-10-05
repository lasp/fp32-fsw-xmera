#include "flybyFilterAlgorithm_c.h"

#include "flybyFilterAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

using FlybyFilterAlgorithm = ::filtering::flybyFilter::FlybyFilterAlgorithm;
using FlybyFilterConfig = ::filtering::flybyFilter::FlybyFilterConfig;
using FlybyFilterOutput = ::filtering::flybyFilter::FlybyFilterOutput;
using FlybyState = ::filtering::flybyFilter::FlybyState;
using HeadingData = ::filtering::flybyFilter::HeadingData;
using StateMatrix = ::filtering::flybyFilter::StateMatrix;

namespace {

FlybyFilterConfig configFromC(const double alpha,
                              const double beta,
                              const double mu,
                              const FlybyFilterStateMatrix_c& processNoiseC,
                              const FlybyFilterStateVector_c& initialStateC,
                              const FlybyFilterStateMatrix_c& initialCovarianceC,
                              const double headingMeasurementNoiseStd) {
    Eigen::Matrix<double, FLYBY_FILTER_NUM_STATES, 1> initialStateVec;
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
    return FlybyFilterConfig::create(
        alpha, beta, mu, processNoise, FlybyState(initialStateVec), initialCovariance, headingMeasurementNoiseStd);
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

FlybyFilterAlgorithmHandle* FlybyFilterAlgorithm_create(double alpha,
                                                        double beta,
                                                        double mu,
                                                        const FlybyFilterStateMatrix_c* processNoise,
                                                        const FlybyFilterStateVector_c* initialState,
                                                        const FlybyFilterStateMatrix_c* initialCovariance,
                                                        double headingMeasurementNoiseStd) {
    return fsw::createHandle<FlybyFilterAlgorithm, FlybyFilterAlgorithmHandle>(
        configFromC(alpha, beta, mu, *processNoise, *initialState, *initialCovariance, headingMeasurementNoiseStd));
}

void FlybyFilterAlgorithm_destroy(FlybyFilterAlgorithmHandle* self) { fsw::deleteHandle<FlybyFilterAlgorithm>(self); }

void FlybyFilterAlgorithm_reInitializeExceptPersistentStates(FlybyFilterAlgorithmHandle* self) {
    fsw::fromHandle<FlybyFilterAlgorithm>(self)->reInitializeExceptPersistentStates();
}

void FlybyFilterAlgorithm_reInitialize(FlybyFilterAlgorithmHandle* self) {
    fsw::fromHandle<FlybyFilterAlgorithm>(self)->reInitialize();
}

FlybyFilterOutput_c FlybyFilterAlgorithm_update(FlybyFilterAlgorithmHandle* self,
                                                double currentSeconds,
                                                const FlybyHeadingData_c* heading) {
    HeadingData in{};
    in.timeTag = heading->timeTag;
    in.rhat_BN_N = Eigen::Vector3d(heading->rhat_BN_N[0], heading->rhat_BN_N[1], heading->rhat_BN_N[2]);

    auto* algo = fsw::fromHandle<FlybyFilterAlgorithm>(self);
    const FlybyFilterOutput out = algo->update(currentSeconds, in);
    return outputToC(out);
}
