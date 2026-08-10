#include "cssWlsEst.h"

#include <utilities/fsw/eigenSupport.h>

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/timeConstants.h"

#include <stdexcept>

static_assert(kMaxNumCss == MAX_NUM_CSS_SENSORS, "kMaxNumCss must match MAX_NUM_CSS_SENSORS");

/*! Number of sun heading components copied into the filter status state vector. */
static constexpr std::size_t kHeadingStateOffset = 0U;

/*! This method performs a complete reset of the module. The configuration properties are pushed into
 the algorithm and all state that retains time varying values between function calls is returned to
 its default.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssWlsEst::reset(const uint64_t callTime) {
    // check that required messages have been included
    if (!this->cssDataInMsg.isLinked()) {
        throw std::invalid_argument("cssWlsEst.cssDataInMsg wasn't connected.");
    }

    if (this->numCss > static_cast<uint32_t>(kMaxNumCss)) {
        throw std::invalid_argument("cssWlsEst.numCss must not be greater than kMaxNumCss.");
    }
    const auto configuredSensors = static_cast<Eigen::Index>(this->numCss);
    if (this->cssNHat.rows() < configuredSensors || this->cssNHat.cols() != 3) {
        throw std::invalid_argument("cssWlsEst.cssNHat must have at least numCss rows and exactly three columns.");
    }
    if (this->cssBias.size() < configuredSensors) {
        throw std::invalid_argument("cssWlsEst.cssBias must have at least numCss entries.");
    }

    this->algorithm.cssNHat_B.setZero();
    this->algorithm.cssBias.setZero();
    this->algorithm.cssNHat_B.topRows(configuredSensors) = this->cssNHat.topRows(configuredSensors);
    this->algorithm.cssBias.head(configuredSensors) = this->cssBias.head(configuredSensors);
    this->algorithm.numCss = this->numCss;
    this->algorithm.useWeights = this->useWeights;
    this->algorithm.sensorUseThresh = this->sensorUseThresh;

    this->algorithm.reInitialize();
    this->numActiveCss = 0;
}

/*! This method reads the CSS array measurements, runs the estimator, and writes the estimated sun
 state along with the post-fit residuals.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CssWlsEst::updateState(const uint64_t callTime) {
    /*! - Read the input parsed CSS sensor data message*/
    const CSSArraySensorMsgF32Payload cssData = this->cssDataInMsg();

    const CssWlsEstOutput out = this->algorithm.update(callTime, cArrayToEigenVector(cssData.CosValue));
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
