#include "inertialFilterAlgorithm_c.h"

#include "inertialFilterAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

using InertialFilterAlgorithm = ::filtering::inertialFilter::InertialFilterAlgorithm;
using InertialFilterConfig = ::filtering::inertialFilter::InertialFilterConfig;
using InertialFilterOutput = ::filtering::inertialFilter::InertialFilterOutput;
using InertialState = ::filtering::inertialFilter::InertialState;
using RateData = ::filtering::inertialFilter::RateData;
using StateMatrix = ::filtering::inertialFilter::StateMatrix;
using StAttData = ::filtering::inertialFilter::StAttData;

namespace {

InertialFilterConfig configFromC(const double alpha,
                                 const double beta,
                                 const InertialFilterStateMatrix_c& processNoiseC,
                                 const InertialFilterStateVector_c& initialStateC,
                                 const InertialFilterStateMatrix_c& initialCovarianceC,
                                 const double stMeasurementNoiseStd,
                                 const double gyroMeasurementNoiseStd) {
    Eigen::Matrix<double, INERTIAL_FILTER_NUM_STATES, 1> initialStateVec;
    for (int i = 0; i < INERTIAL_FILTER_NUM_STATES; ++i) {
        initialStateVec(i) = initialStateC.data[i];
    }
    StateMatrix processNoise;
    StateMatrix initialCovariance;
    for (int i = 0; i < INERTIAL_FILTER_NUM_STATES; ++i) {
        for (int j = 0; j < INERTIAL_FILTER_NUM_STATES; ++j) {
            processNoise(i, j) = processNoiseC.data[(i * INERTIAL_FILTER_NUM_STATES) + j];
            initialCovariance(i, j) = initialCovarianceC.data[(i * INERTIAL_FILTER_NUM_STATES) + j];
        }
    }
    return InertialFilterConfig::create(alpha,
                                        beta,
                                        processNoise,
                                        InertialState(initialStateVec),
                                        initialCovariance,
                                        stMeasurementNoiseStd,
                                        gyroMeasurementNoiseStd);
}

// StAttResidualsOutput and RateResidualsOutput are distinct C++ types with identical fields; one
// converter serves both.
template <class Residuals>
InertialResiduals_c residualsToC(const Residuals& r) {
    InertialResiduals_c out{};
    out.valid = r.valid;
    for (int i = 0; i < 3; ++i) {
        out.observation[i] = r.observation(i);
        out.preFit[i] = r.preFit(i);
        out.postFit[i] = r.postFit(i);
    }
    return out;
}

InertialFilterOutput_c outputToC(const InertialFilterOutput& out) {
    InertialFilterOutput_c result{};
    for (int i = 0; i < INERTIAL_FILTER_NUM_STATES; ++i) {
        result.state[i] = out.filterState.state(i);
        for (int j = 0; j < INERTIAL_FILTER_NUM_STATES; ++j) {
            result.covariance[i][j] = out.filterState.covariance(i, j);
        }
    }
    result.stAttResiduals = residualsToC(out.stAttResiduals);
    result.rateResiduals = residualsToC(out.rateResiduals);
    return result;
}

}  // namespace

uint32_t InertialFilterAlgorithm_getNumStates(void) { return INERTIAL_FILTER_NUM_STATES; }

bool InertialFilterAlgorithm_validateConfig(double alpha,
                                            double beta,
                                            const InertialFilterStateMatrix_c* processNoise,
                                            const InertialFilterStateVector_c* initialState,
                                            const InertialFilterStateMatrix_c* initialCovariance,
                                            double stMeasurementNoiseStd,
                                            double gyroMeasurementNoiseStd) {
    // Build the config through the same path create() uses: success means valid, a throw means
    // invalid. Sharing configFromC keeps the predicate from drifting from what create() accepts.
    try {
        (void)configFromC(alpha,
                          beta,
                          *processNoise,
                          *initialState,
                          *initialCovariance,
                          stMeasurementNoiseStd,
                          gyroMeasurementNoiseStd);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

InertialFilterAlgorithmHandle* InertialFilterAlgorithm_create(double alpha,
                                                              double beta,
                                                              const InertialFilterStateMatrix_c* processNoise,
                                                              const InertialFilterStateVector_c* initialState,
                                                              const InertialFilterStateMatrix_c* initialCovariance,
                                                              double stMeasurementNoiseStd,
                                                              double gyroMeasurementNoiseStd) {
    return fsw::createHandle<InertialFilterAlgorithm, InertialFilterAlgorithmHandle>(configFromC(
        alpha, beta, *processNoise, *initialState, *initialCovariance, stMeasurementNoiseStd, gyroMeasurementNoiseStd));
}

void InertialFilterAlgorithm_destroy(InertialFilterAlgorithmHandle* self) {
    fsw::deleteHandle<InertialFilterAlgorithm>(self);
}

void InertialFilterAlgorithm_reInitializeExceptPersistentStates(InertialFilterAlgorithmHandle* self) {
    fsw::fromHandle<InertialFilterAlgorithm>(self)->reInitializeExceptPersistentStates();
}

void InertialFilterAlgorithm_reInitialize(InertialFilterAlgorithmHandle* self) {
    fsw::fromHandle<InertialFilterAlgorithm>(self)->reInitialize();
}

InertialFilterOutput_c InertialFilterAlgorithm_update(InertialFilterAlgorithmHandle* self,
                                                      double currentSeconds,
                                                      const StAttData_c* stAtt,
                                                      const RateData_c* rate) {
    StAttData stIn{};
    stIn.timeTag = stAtt->timeTag;
    stIn.sigma_BN = Eigen::Vector3d(stAtt->sigma_BN[0], stAtt->sigma_BN[1], stAtt->sigma_BN[2]);

    RateData rateIn{};
    rateIn.timeTag = rate->timeTag;
    rateIn.rate = Eigen::Vector3d(rate->rate[0], rate->rate[1], rate->rate[2]);

    auto* algo = fsw::fromHandle<InertialFilterAlgorithm>(self);
    const InertialFilterOutput out = algo->update(currentSeconds, stIn, rateIn);
    return outputToC(out);
}
