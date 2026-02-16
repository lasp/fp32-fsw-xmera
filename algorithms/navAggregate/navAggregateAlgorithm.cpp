#include "navAggregateAlgorithm.h"

#include "../freestandingInvalidArgument.h"
#include <algorithm>
#include <stdio.h>

/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return AggregateOutput attitude navigation and translation navigation output
 @param attInputs Aggregated attitude navigation inputs
 @param transInputs Aggregated translational navigation inputs
 */
AggregateOutput NavAggregateAlgorithm::update(
    std::array<InputNavAttData, MAX_AGG_NAV_MSG> attInputs,
    std::array<InputNavTransData, MAX_AGG_NAV_MSG> transInputs) const {
    InputNavAttData navAttOutput{};      /* [-] local storage of the outgoing attitude navigation data*/
    InputNavTransData navTransOutput{};  /* [-] local storage of the outgoing translation navigation data*/

    /*! - check that attitude navigation messages are present */
    if (this->attMsgCount) {
        /*! - Copy out each part of the attitude source message into the target output message*/
        navAttOutput.timeTag = attInputs.at(this->attTimeIdx).timeTag;
        navAttOutput.sigma_BN = attInputs.at(this->attIdx).sigma_BN;
        navAttOutput.omega_BN_B = attInputs.at(this->rateIdx).omega_BN_B;
        navAttOutput.vehSunPntBdy = attInputs.at(this->sunIdx).vehSunPntBdy;
    }

    /*! - check that translation navigation messages are present */
    if (this->transMsgCount) {
        /*! - Copy out each part of the translation source message into the target output message*/
        navTransOutput.timeTag = transInputs.at(this->transTimeIdx).timeTag;
        navTransOutput.r_BN_N = transInputs.at(this->posIdx).r_BN_N;
        navTransOutput.v_BN_N = transInputs.at(this->velIdx).v_BN_N;
        navTransOutput.vehAccumDV = transInputs.at(this->dvIdx).vehAccumDV;
    }

    AggregateOutput navAggregateOut{};
    navAggregateOut.navAttOut = navAttOutput;
    navAggregateOut.navTransOut = navTransOutput;

    return navAggregateOut;
}

/**
 * @brief Set the attitude time index.
 * @param idx The new attitude time index to set.
 */
void NavAggregateAlgorithm::setAttTimeIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "attTimeIdx (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 idx,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->attTimeIdx = idx;
}

/**
 * @brief Get the attitude time index.
 * @return uint32_t The current attitude time index.
 */
uint32_t NavAggregateAlgorithm::getAttTimeIdx() const { return this->attTimeIdx; }

/**
 * @brief Set the translation time index.
 * @param idx The new translation time index to set.
 */
void NavAggregateAlgorithm::setTransTimeIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "transTimeIdx (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 idx,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->transTimeIdx = idx;
}

/**
 * @brief Get the translation time index.
 * @return uint32_t The current translation time index.
 */
uint32_t NavAggregateAlgorithm::getTransTimeIdx() const { return this->transTimeIdx; }

/**
 * @brief Set the attitude index.
 * @param idx The new attitude index to set.
 */
void NavAggregateAlgorithm::setAttIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "attIdx (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 idx,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->attIdx = idx;
}

/**
 * @brief Get the attitude index.
 * @return uint32_t The current attitude index.
 */
uint32_t NavAggregateAlgorithm::getAttIdx() const { return this->attIdx; }

/**
 * @brief Set the rate index.
 * @param idx The new rate index to set.
 */
void NavAggregateAlgorithm::setRateIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "rateIdx (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 idx,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->rateIdx = idx;
}

/**
 * @brief Get the rate index.
 * @return uint32_t The current rate index.
 */
uint32_t NavAggregateAlgorithm::getRateIdx() const { return this->rateIdx; }

/**
 * @brief Set the position index.
 * @param idx The new position index to set.
 */
void NavAggregateAlgorithm::setPosIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "posIdx (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 idx,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->posIdx = idx;
}

/**
 * @brief Get the position index.
 * @return uint32_t The current position index.
 */
uint32_t NavAggregateAlgorithm::getPosIdx() const { return this->posIdx; }

/**
 * @brief Set the velocity index.
 * @param idx The new velocity index to set.
 */
void NavAggregateAlgorithm::setVelIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "velIdx (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 idx,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->velIdx = idx;
}

/**
 * @brief Get the velocity index.
 * @return uint32_t The current velocity index.
 */
uint32_t NavAggregateAlgorithm::getVelIdx() const { return this->velIdx; }

/**
 * @brief Set the accumulated DV index.
 * @param idx The new accumulated DV index to set.
 */
void NavAggregateAlgorithm::setDvIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "dvIdx (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 idx,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->dvIdx = idx;
}

/**
 * @brief Get the accumulated DV index.
 * @return uint32_t The current accumulated DV index.
 */
uint32_t NavAggregateAlgorithm::getDvIdx() const { return this->dvIdx; }

/**
 * @brief Set the sun index.
 * @param idx The new sun index to set.
 */
void NavAggregateAlgorithm::setSunIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "attMsgCount (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 idx,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->sunIdx = idx;
}

/**
 * @brief Get the sun index.
 * @return uint32_t The current sun index.
 */
uint32_t NavAggregateAlgorithm::getSunIdx() const { return this->sunIdx; }

/**
 * @brief Set the attitude message count.
 * @param msgCount The new attitude message count to set.
 */
void NavAggregateAlgorithm::setAttMsgCount(uint32_t msgCount) {
    if (msgCount > MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "attMsgCount (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 msgCount,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->attMsgCount = msgCount;
}

/**
 * @brief Get the attitude message count.
 * @return uint32_t The current attitude message count.
 */
uint32_t NavAggregateAlgorithm::getAttMsgCount() const { return this->attMsgCount; }

/**
 * @brief Set the translational message count.
 * @param msgCount The new translational message count to set.
 */
void NavAggregateAlgorithm::setTransMsgCount(uint32_t msgCount) {
    if (msgCount > MAX_AGG_NAV_MSG) {
        char msg[128];
        snprintf(msg,
                 128,
                 "transMsgCount (%u) must be less than maximum navAggregate "
                 "message size (%d)",
                 msgCount,
                 MAX_AGG_NAV_MSG);
        FS_THROW_INVALID_ARGUMENT(msg);
    }
    this->transMsgCount = msgCount;
}

/**
 * @brief Get the translational message count.
 * @return uint32_t The current translational message count.
 */
uint32_t NavAggregateAlgorithm::getTransMsgCount() const { return this->transMsgCount; }
