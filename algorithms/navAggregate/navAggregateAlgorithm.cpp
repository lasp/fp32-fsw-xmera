#include "navAggregateAlgorithm.h"

#include "../freestandingInvalidArgument.h"
#include <stdio.h>

/**
 * @brief Copy a 3x1 vector.
 * @param v The vector to be copied.
 * @param result The vector copy.
 */
template <typename ScalarT>
static void v3Copy(ScalarT const v[3], ScalarT result[3]) {
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
}

/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param attMsgsPayloads Aggregated attitude navigation messages
 @param transMsgsPayloads Aggregated translational navigation messages
 */
AggregateOutput NavAggregateAlgorithm::update(std::array<NavAttMsgF32Payload, MAX_AGG_NAV_MSG> attMsgsPayloads,
                                              std::array<NavTransMsgF32Payload, MAX_AGG_NAV_MSG> transMsgsPayloads) const {
    NavAttMsgF32Payload navAttOutMsgPayload{}; /* [-] local storage of the outgoing attitude navigation message data*/
    NavTransMsgF32Payload
        navTransOutMsgPayload{}; /* [-] local storage of the outgoing translation navigation message data*/
    AggregateOutput navAggregateOut{};

    /*! - check that attitude navigation messages are present */
    if (this->attMsgCount) {
        /*! - Copy out each part of the attitude source message into the target output message*/
        navAttOutMsgPayload.timeTag = attMsgsPayloads[this->attTimeIdx].timeTag;
        v3Copy(attMsgsPayloads[this->attIdx].sigma_BN, navAttOutMsgPayload.sigma_BN);
        v3Copy(attMsgsPayloads[this->rateIdx].omega_BN_B, navAttOutMsgPayload.omega_BN_B);
        v3Copy(attMsgsPayloads[this->sunIdx].vehSunPntBdy, navAttOutMsgPayload.vehSunPntBdy);
    }

    /*! - check that translation navigation messages are present */
    if (this->transMsgCount) {
        /*! - Copy out each part of the translation source message into the target output message*/
        navTransOutMsgPayload.timeTag = transMsgsPayloads[this->transTimeIdx].timeTag;
        v3Copy(transMsgsPayloads[this->posIdx].r_BN_N, navTransOutMsgPayload.r_BN_N);
        v3Copy(transMsgsPayloads[this->velIdx].v_BN_N, navTransOutMsgPayload.v_BN_N);
        v3Copy(transMsgsPayloads[this->dvIdx].vehAccumDV, navTransOutMsgPayload.vehAccumDV);
    }

    navAggregateOut.navAttOut = navAttOutMsgPayload;
    navAggregateOut.navTransOut = navTransOutMsgPayload;

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
