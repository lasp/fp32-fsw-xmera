#include "navAggregate.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <array>
#include <stdexcept>

using namespace f32;

/*! This resets the module to original states. It builds and validates the immutable configuration from the
    adapter's selection properties and verifies the configured input messages are linked.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::reset(uint64_t callTime) {
    const NavAggregateAttSelection attSelection{
        .attTimeIdx = this->attTimeIdx,
        .attIdx = this->attIdx,
        .rateIdx = this->rateIdx,
        .sunIdx = this->sunIdx,
        .attMsgCount = this->attMsgCount,
    };
    const NavAggregateTransSelection transSelection{
        .transTimeIdx = this->transTimeIdx,
        .posIdx = this->posIdx,
        .velIdx = this->velIdx,
        .dvIdx = this->dvIdx,
        .transMsgCount = this->transMsgCount,
    };
    const NavAggregateConfig config = NavAggregateConfig::create(attSelection, transSelection);
    this->algorithm = std::make_unique<NavAggregateAlgorithm>(config);

    /*! - loop over the number of attitude input messages and make sure they are linked */
    for (uint32_t i = 0U; i < this->attMsgCount; ++i) {
        if (!this->attMsgs[i].navAttInMsg.isLinked()) {
            throw std::invalid_argument(
                "An attitude input message name was not linked. "
                "Be sure that the number of linked messages corresponds to attMsgCount.");
        }
    }
    /*! - loop over the number of translational input messages and make sure they are linked */
    for (uint32_t i = 0U; i < this->transMsgCount; ++i) {
        if (!this->transMsgs[i].navTransInMsg.isLinked()) {
            throw std::invalid_argument(
                "A translation input message name was not linked. "
                "Be sure that the number of linked messages corresponds to transMsgCount.");
        }
    }

    //! - zero the arrays of input messages
    for (uint32_t i = 0U; i < MAX_AGG_NAV_MSG; ++i) {
        this->attMsgs[i].msgStorage = NavAttMsgF32Payload();
        this->transMsgs[i].msgStorage = NavTransMsgF32Payload();
    }
}

NavAggregateConfig NavAggregate::toConfig() const {
    const NavAggregateAttSelection attSelection{
        .attTimeIdx = this->attTimeIdx,
        .attIdx = this->attIdx,
        .rateIdx = this->rateIdx,
        .sunIdx = this->sunIdx,
        .attMsgCount = this->attMsgCount,
    };
    const NavAggregateTransSelection transSelection{
        .transTimeIdx = this->transTimeIdx,
        .posIdx = this->posIdx,
        .velIdx = this->velIdx,
        .dvIdx = this->dvIdx,
        .transMsgCount = this->transMsgCount,
    };
    return NavAggregateConfig::create(attSelection, transSelection);
}

void NavAggregate::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("NavAggregate reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("NavAggregate reset() has not been called.");
    }

    std::array<InputNavAttData, MAX_AGG_NAV_MSG> attInputs{};
    std::array<InputNavTransData, MAX_AGG_NAV_MSG> transInputs{};

    /*! - check that attitude navigation messages are present */
    if (this->attMsgCount > 0U) {
        /*! - Iterate through all of the attitude input messages, clear local Msg buffer and archive the new nav data */
        for (uint32_t i = 0U; i < this->attMsgCount; ++i) {
            this->attMsgs[i].msgStorage = this->attMsgs[i].navAttInMsg();
            NavAttMsgF32Payload navAttMsgPayload = this->attMsgs[i].navAttInMsg();

            InputNavAttData attInputData{};
            attInputData.timeTag = navAttMsgPayload.timeTag;
            attInputData.sigma_BN = cArrayToEigenVector(navAttMsgPayload.sigma_BN);
            attInputData.omega_BN_B = cArrayToEigenVector(navAttMsgPayload.omega_BN_B);
            attInputData.vehSunPntBdy = cArrayToEigenVector(navAttMsgPayload.vehSunPntBdy);
            attInputs.at(i) = attInputData;
        }
    }

    /*! - check that translation navigation messages are present */
    if (this->transMsgCount > 0U) {
        /*! - Iterate through all of the translation input messages, clear local Msg buffer and archive the new nav data
         */
        for (uint32_t i = 0U; i < this->transMsgCount; ++i) {
            this->transMsgs[i].msgStorage = this->transMsgs[i].navTransInMsg();
            NavTransMsgF32Payload navTransMsgPayload = this->transMsgs[i].navTransInMsg();

            InputNavTransData transInputData{};
            transInputData.timeTag = navTransMsgPayload.timeTag;
            transInputData.r_BN_N = cArrayToEigenVector(navTransMsgPayload.r_BN_N);
            transInputData.v_BN_N = cArrayToEigenVector(navTransMsgPayload.v_BN_N);
            transInputData.vehAccumDV = cArrayToEigenVector(navTransMsgPayload.vehAccumDV);
            transInputs.at(i) = transInputData;
        }
    }

    AggregateOutput navAggregateOut = this->algorithm->update(attInputs, transInputs);

    NavAttMsgF32Payload navAttOutMsgPayload{};
    navAttOutMsgPayload.timeTag = navAggregateOut.navAttOut.timeTag;
    eigenVectorToCArray(navAggregateOut.navAttOut.sigma_BN, navAttOutMsgPayload.sigma_BN);
    eigenVectorToCArray(navAggregateOut.navAttOut.omega_BN_B, navAttOutMsgPayload.omega_BN_B);
    eigenVectorToCArray(navAggregateOut.navAttOut.vehSunPntBdy, navAttOutMsgPayload.vehSunPntBdy);

    NavTransMsgF32Payload navTransOutMsgPayload{};
    navTransOutMsgPayload.timeTag = navAggregateOut.navTransOut.timeTag;
    eigenVectorToCArray(navAggregateOut.navTransOut.r_BN_N, navTransOutMsgPayload.r_BN_N);
    eigenVectorToCArray(navAggregateOut.navTransOut.v_BN_N, navTransOutMsgPayload.v_BN_N);
    eigenVectorToCArray(navAggregateOut.navTransOut.vehAccumDV, navTransOutMsgPayload.vehAccumDV);

    this->navAttOutMsg.write(navAttOutMsgPayload, this->moduleID, callTime);
    this->navTransOutMsg.write(navTransOutMsgPayload, this->moduleID, callTime);
}
