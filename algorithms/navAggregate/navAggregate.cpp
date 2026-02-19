#include "navAggregate.h"
#include "architecture/utilities/eigenSupport.h"
#include <array>
#include <stdexcept>

/*! This resets the module to original states.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::reset(uint64_t callTime) {
    /*! - loop over the number of attitude input messages and make sure they are linked */
    for (uint32_t i = 0U; i < this->getAttMsgCount(); ++i) {
        if (!this->attMsgs[i].navAttInMsg.isLinked()) {
            throw std::invalid_argument(
                "An attitude input message name was not linked. "
                "Be sure that the number of linked messages corresponds to attMsgCount.");
        }
    }
    /*! - loop over the number of translational input messages and make sure they are linked */
    for (uint32_t i = 0U; i < this->getTransMsgCount(); ++i) {
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

/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::updateState(uint64_t callTime) {
    std::array<InputNavAttData, MAX_AGG_NAV_MSG> attInputs{};
    std::array<InputNavTransData, MAX_AGG_NAV_MSG> transInputs{};

    /*! - check that attitude navigation messages are present */
    if (this->getAttMsgCount() > 0U) {
        /*! - Iterate through all of the attitude input messages, clear local Msg buffer and archive the new nav data */
        for (uint32_t i = 0U; i < this->getAttMsgCount(); ++i) {
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
    if (this->getTransMsgCount() > 0U) {
        /*! - Iterate through all of the translation input messages, clear local Msg buffer and archive the new nav data
         */
        for (uint32_t i = 0U; i < this->getTransMsgCount(); ++i) {
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

    AggregateOutput navAggregateOut = this->algorithm.update(attInputs, transInputs);

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

    this->navAttOutMsg.write(&navAttOutMsgPayload, this->moduleID, callTime);
    this->navTransOutMsg.write(&navTransOutMsgPayload, this->moduleID, callTime);
}

/**
 * @brief Set the attitude time index.
 * @param idx The new attitude time index to set.
 */
void NavAggregate::setAttTimeIdx(uint32_t idx) { this->algorithm.setAttTimeIdx(idx); }

/**
 * @brief Get the attitude time index.
 * @return uint32_t The current attitude time index.
 */
uint32_t NavAggregate::getAttTimeIdx() const { return this->algorithm.getAttTimeIdx(); }

/**
 * @brief Set the translation time index.
 * @param idx The new translation time index to set.
 */
void NavAggregate::setTransTimeIdx(uint32_t idx) { this->algorithm.setTransTimeIdx(idx); }

/**
 * @brief Get the translation time index.
 * @return uint32_t The current translation time index.
 */
uint32_t NavAggregate::getTransTimeIdx() const { return this->algorithm.getTransTimeIdx(); }

/**
 * @brief Set the attitude index.
 * @param idx The new attitude index to set.
 */
void NavAggregate::setAttIdx(uint32_t idx) { this->algorithm.setAttIdx(idx); }

/**
 * @brief Get the attitude index.
 * @return uint32_t The current attitude index.
 */
uint32_t NavAggregate::getAttIdx() const { return this->algorithm.getAttIdx(); }

/**
 * @brief Set the rate index.
 * @param idx The new rate index to set.
 */
void NavAggregate::setRateIdx(uint32_t idx) { this->algorithm.setRateIdx(idx); }

/**
 * @brief Get the rate index.
 * @return uint32_t The current rate index.
 */
uint32_t NavAggregate::getRateIdx() const { return this->algorithm.getRateIdx(); }

/**
 * @brief Set the position index.
 * @param idx The new position index to set.
 */
void NavAggregate::setPosIdx(uint32_t idx) { this->algorithm.setPosIdx(idx); }

/**
 * @brief Get the position index.
 * @return uint32_t The current position index.
 */
uint32_t NavAggregate::getPosIdx() const { return this->algorithm.getPosIdx(); }

/**
 * @brief Set the velocity index.
 * @param idx The new velocity index to set.
 */
void NavAggregate::setVelIdx(uint32_t idx) { this->algorithm.setVelIdx(idx); }

/**
 * @brief Get the velocity index.
 * @return uint32_t The current velocity index.
 */
uint32_t NavAggregate::getVelIdx() const { return this->algorithm.getVelIdx(); }

/**
 * @brief Set the accumulated DV index.
 * @param idx The new accumulated DV index to set.
 */
void NavAggregate::setDvIdx(uint32_t idx) { this->algorithm.setDvIdx(idx); }

/**
 * @brief Get the accumulated DV index.
 * @return uint32_t The current accumulated DV index.
 */
uint32_t NavAggregate::getDvIdx() const { return this->algorithm.getDvIdx(); }

/**
 * @brief Set the sun index.
 * @param idx The new sun index to set.
 */
void NavAggregate::setSunIdx(uint32_t idx) { this->algorithm.setSunIdx(idx); }

/**
 * @brief Get the sun index.
 * @return uint32_t The current sun index.
 */
uint32_t NavAggregate::getSunIdx() const { return this->algorithm.getSunIdx(); }

/**
 * @brief Set the attitude message count.
 * @param msgCount The new attitude message count to set.
 */
void NavAggregate::setAttMsgCount(uint32_t msgCount) { this->algorithm.setAttMsgCount(msgCount); }

/**
 * @brief Get the attitude message count.
 * @return uint32_t The current attitude message count.
 */
uint32_t NavAggregate::getAttMsgCount() const { return this->algorithm.getAttMsgCount(); }

/**
 * @brief Set the translational message count.
 * @param msgCount The new translational message count to set.
 */
void NavAggregate::setTransMsgCount(uint32_t msgCount) { this->algorithm.setTransMsgCount(msgCount); }

/**
 * @brief Get the translational message count.
 * @return uint32_t The current translational message count.
 */
uint32_t NavAggregate::getTransMsgCount() const { return this->algorithm.getTransMsgCount(); }
