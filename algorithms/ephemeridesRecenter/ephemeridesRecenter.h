#ifndef F32XIMERA_EPHEM_RECENTER_H
#define F32XIMERA_EPHEM_RECENTER_H

#include "ephemeridesRecenterAlgorithm.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <Eigen/Core>
#include <array>

/*! @brief Container class for the input and output messages that will be re-centered on the base ephemeris. */
class BodyEphemeris {
   public:
    int bodySpiceId{};
    int originalCentralBodyId{};
    ReadFunctor<EphemerisMsgF32Payload> inputEphemerisMsg{};
    Message<EphemerisMsgF32Payload> outputEphemerisMsg{};
};

/*! @brief Ephemerides Recenter Basilisk Model */
class EphemeridesRecenter : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    void addBodyEphemerisToRecenter(const BodyEphemeris& ephemerisBody);
    void setNewZeroBase(int bodySpiceId);
    int getNewZeroBase() const;
    void setPreviousCommonZeroBase(int bodySpiceId);
    int getPreviousCommonZeroBase() const;
    std::array<int, MAX_NUM_CHANGE_BODIES> getAllIds() const;
    size_t getNumberOfBodies() const;
    void clearAllBodies();
    size_t findBodyIndex(int bodySpiceId) const;
    std::vector<Message<EphemerisMsgF32Payload>*> recenteredEphemerisOutputMsgs{};

   private:
    size_t ephemeridesNumber{};
    std::array<BodyEphemeris, MAX_NUM_CHANGE_BODIES> ephemerides{};
    EphemeridesRecenterAlgorithm algorithm{};
};

#endif
