#ifndef F32XMERA_NAV_AGGREGATE_H
#define F32XMERA_NAV_AGGREGATE_H

#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "navAggregateAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>

namespace f32 {

/*! structure containing the attitude navigation message name, ID and local buffer*/
typedef struct {
    ReadFunctor<NavAttMsgF32Payload> navAttInMsg; /*!< attitude navigation input message*/
    NavAttMsgF32Payload msgStorage;               /*!< [-] Local buffer to store nav message*/
} AggregateAttInput;

/*! structure containing the translational navigation message name, ID and local buffer*/
typedef struct {
    ReadFunctor<NavTransMsgF32Payload> navTransInMsg; /*!< translation navigation input message*/
    NavTransMsgF32Payload msgStorage;                 /*!< [-] Local buffer to store nav message*/
} AggregateTransInput;

class NavAggregate : public SysModel {
   public:
    NavAggregate() = default;
    ~NavAggregate() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setAttTimeIdx(uint32_t idx);
    uint32_t getAttTimeIdx() const;
    void setTransTimeIdx(uint32_t idx);
    uint32_t getTransTimeIdx() const;
    void setAttIdx(uint32_t idx);
    uint32_t getAttIdx() const;
    void setRateIdx(uint32_t idx);
    uint32_t getRateIdx() const;
    void setPosIdx(uint32_t idx);
    uint32_t getPosIdx() const;
    void setVelIdx(uint32_t idx);
    uint32_t getVelIdx() const;
    void setDvIdx(uint32_t idx);
    uint32_t getDvIdx() const;
    void setSunIdx(uint32_t idx);
    uint32_t getSunIdx() const;
    void setAttMsgCount(uint32_t msgCount);
    uint32_t getAttMsgCount() const;
    void setTransMsgCount(uint32_t msgCount);
    uint32_t getTransMsgCount() const;

    AggregateAttInput attMsgs[MAX_AGG_NAV_MSG];     /*!< [-] The incoming nav message buffer */
    AggregateTransInput transMsgs[MAX_AGG_NAV_MSG]; /*!< [-] The incoming nav message buffer */
    Message<NavAttMsgF32Payload> navAttOutMsg;      /*!< blended attitude navigation output message */
    Message<NavTransMsgF32Payload> navTransOutMsg;  /*!< blended translation navigation output message */

   private:
    NavAggregateAlgorithm algorithm{};
};

}  // namespace f32

#endif
