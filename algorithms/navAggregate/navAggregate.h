#ifndef F32XMERA_NAV_AGGREGATE_H
#define F32XMERA_NAV_AGGREGATE_H

#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "navAggregateAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

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

class NavAggregate final : public SysModel {
   public:
    NavAggregate() = default;
    ~NavAggregate() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure() const;

    // Phase 1: public config properties -- set before reset().
    uint32_t attTimeIdx{};     //!< [-] index of the message providing the attitude message time
    uint32_t transTimeIdx{};   //!< [-] index of the message providing the translation message time
    uint32_t attIdx{};         //!< [-] index of the message providing the inertial MRP
    uint32_t rateIdx{};        //!< [-] index of the message providing the attitude rate
    uint32_t posIdx{};         //!< [-] index of the message providing the inertial position
    uint32_t velIdx{};         //!< [-] index of the message providing the inertial velocity
    uint32_t dvIdx{};          //!< [-] index of the message providing the accumulated DV
    uint32_t sunIdx{};         //!< [-] index of the message providing the sun-pointing vector
    uint32_t attMsgCount{};    //!< [-] number of attitude messages available as inputs
    uint32_t transMsgCount{};  //!< [-] number of translation messages available as inputs

    AggregateAttInput attMsgs[MAX_AGG_NAV_MSG];     /*!< [-] The incoming nav message buffer */
    AggregateTransInput transMsgs[MAX_AGG_NAV_MSG]; /*!< [-] The incoming nav message buffer */
    Message<NavAttMsgF32Payload> navAttOutMsg;      /*!< blended attitude navigation output message */
    Message<NavTransMsgF32Payload> navTransOutMsg;  /*!< blended translation navigation output message */

   private:
    NavAggregateConfig toConfig() const;
    std::unique_ptr<NavAggregateAlgorithm> algorithm = nullptr;
};

}  // namespace f32

#endif
