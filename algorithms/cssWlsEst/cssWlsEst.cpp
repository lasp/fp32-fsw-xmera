#include "cssWlsEst.h"

#include "utilities/xmera/xmeraLifecycleException.h"
#include <utilities/fsw/eigenSupport.h>

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/timeConstants.h"

#include <memory>
#include <stdexcept>

static_assert(kMaxNumCss == MAX_NUM_CSS_SENSORS, "kMaxNumCss must match MAX_NUM_CSS_SENSORS");

/*! Index in the filter status state vector at which the sun heading is written. */
static constexpr std::size_t kHeadingStateOffset = 0U;

/*! Number of columns expected in the cssNHat property, one per body frame component. */
static constexpr Eigen::Index kBoresightComponents = 3;

/*! Validate the message connections and construct the algorithm from the public properties. Startup
 only; on a state transition the flight software calls reInitialize() instead.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssWlsEst::reset(const uint64_t callTime) {
    // check that required messages have been included
    if (!this->cssDataInMsg.isLinked()) {
        throw std::invalid_argument("cssWlsEst.cssDataInMsg wasn't connected.");
    }

    this->algorithm = std::make_unique<CssWlsEstAlgorithm>(this->toConfig());
    this->numActiveCss = 0;
}

/*! Build a validated algorithm configuration from the current module properties. The dynamically sized
 properties are checked here and packed into the algorithm's fixed-size types.
 @return CssWlsEstConfig validated configuration
 */
CssWlsEstConfig CssWlsEst::toConfig() const {
    if (this->numCss > static_cast<uint32_t>(kMaxNumCss)) {
        throw std::invalid_argument("cssWlsEst.numCss must not be greater than kMaxNumCss.");
    }
    const auto configuredSensors = static_cast<Eigen::Index>(this->numCss);
    if (this->cssNHat.rows() < configuredSensors || this->cssNHat.cols() != kBoresightComponents) {
        throw std::invalid_argument("cssWlsEst.cssNHat must have at least numCss rows and exactly three columns.");
    }
    if (this->cssBias.size() < configuredSensors) {
        throw std::invalid_argument("cssWlsEst.cssBias must have at least numCss entries.");
    }

    Eigen::Matrix<float, kMaxNumCss, 3> cssNHat_B = Eigen::Matrix<float, kMaxNumCss, 3>::Zero();
    Eigen::Vector<float, kMaxNumCss> cssBiasPacked = Eigen::Vector<float, kMaxNumCss>::Zero();
    cssNHat_B.topRows(configuredSensors) = this->cssNHat.topRows(configuredSensors);
    cssBiasPacked.head(configuredSensors) = this->cssBias.head(configuredSensors);

    return CssWlsEstConfig::create(cssNHat_B, cssBiasPacked, this->numCss, this->useWeights, this->sensorUseThresh);
}

/*! Re-validate the current module properties and push them onto the live algorithm, leaving the
 estimator's runtime state untouched.
 @return void
 */
void CssWlsEst::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("CssWlsEst reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Clear the estimator's runtime state; a pass-through to the algorithm's reInitialize(). The prior
 heading, the prior-signal flag and the prior time are all non-persistent, so a mode transition produces
 no rate until two headings have been seen again.
 @return void
 */
void CssWlsEst::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("CssWlsEst reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! This method reads the CSS array measurements, runs the estimator, and writes the estimated sun
 state along with the post-fit residuals.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssWlsEst::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("CssWlsEst reset() has not been called.");
    }

    /*! - Read the input parsed CSS sensor data message*/
    const CSSArraySensorMsgF32Payload cssData = this->cssDataInMsg();

    const CssWlsEstOutput out = this->algorithm->update(callTime, cArrayToEigenVector(cssData.CosValue));
    this->numActiveCss = out.numActiveCss;

    /*! - If the residual fit output message is set, then store the residuals in the output message */
    if (this->cssWLSFiltResOutMsg.isLinked()) {
        SunlineFilterMsgF32Payload filtStatus = {};
        filtStatus.numObs = static_cast<int>(out.numActiveCss);
        filtStatus.timeTag = static_cast<double>(callTime) * kNano2Sec;
        eigenMatrixXInsertCArray(out.residualStateHeading, filtStatus.state, kHeadingStateOffset);
        eigenVectorToCArray(out.postFitResiduals, filtStatus.postFitRes);
        this->cssWLSFiltResOutMsg.write(filtStatus, this->moduleID, callTime);
    }

    /*! - Populate the navigation output message with the estimated sun state */
    NavAttMsgF32Payload sunlineOutBuffer = {};
    eigenVectorToCArray(out.sunHeading_B, sunlineOutBuffer.vehSunPntBdy);
    eigenVectorToCArray(out.omega_BN_B, sunlineOutBuffer.omega_BN_B);
    this->navStateOutMsg.write(sunlineOutBuffer, this->moduleID, callTime);
}
