#ifndef F32XIMERA_OE_STATE_EPHEM_H
#define F32XIMERA_OE_STATE_EPHEM_H

#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "oeStateEphemAlgorithm.h"
#include <architecture/msgPayloadDef/TDBVehicleClockCorrelationMsgPayload.h>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <memory>

/*! @brief Top level structure for the Chebyshev position ephemeris
           fit system.  Allows the user to specify a set of chebyshev
           coefficients and then use the input time to determine where
           a given body is in space
*/
class OEStateEphem : public SysModel {
   public:
    OEStateEphem() = default;
    ~OEStateEphem() override = default;

    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    Message<EphemerisMsgF32Payload> stateFitOutMsg;                    //!< [-] output navigation message for pos/vel
    ReadFunctor<TDBVehicleClockCorrelationMsgPayload> clockCorrInMsg;  //!< clock correlation input message

    double centralBodyGravitationalParameter{};  //!< [m^3/s^2] gravitational parameter of the central body
    unsigned int numberOfArcs{1};                //!< [-] number of populated Chebyshev fit arcs
    int spiceBodyId{};                           //!< [-] SPICE ID written to the output ephemeris message
    int centralBodyId{};                         //!< [-] central-body SPICE ID written to the output message

    void setArcNumberOfCoefficients(unsigned int arcNumber, unsigned int numberOfCoefficients);
    unsigned int getArcNumberOfCoefficients(unsigned int arcNumber) const;

    void setArcMiddleTime(unsigned int arcNumber, double timeMiddle);
    double getArcMiddleTime(unsigned int arcNumber) const;

    void setArcRadiusTime(unsigned int arcNumber, double timeRadius);
    double getArcRadiusTime(unsigned int arcNumber) const;

    void setArcAnomalyFlag(unsigned int arcNumber, const AnomalyType& anomalyFlag);
    AnomalyType getArcAnomalyFlag(unsigned int arcNumber) const;

    void setArcRadiusPeriapsisCoefficients(unsigned int arcNumber,
                                           const std::array<double, kMaxOeCoeff>& radiusPeriapsisCoefficients);
    std::array<double, kMaxOeCoeff> getArcRadiusPeriapsisCoefficients(unsigned int arcNumber);
    void setArcEccentricityCoefficients(unsigned int arcNumber,
                                        const std::array<double, kMaxOeCoeff>& eccentricityCoefficients);
    std::array<double, kMaxOeCoeff> getArcEccentricityCoefficients(unsigned int arcNumber);
    void setArcInclinationCoefficients(unsigned int arcNumber,
                                       const std::array<double, kMaxOeCoeff>& inclinationCoefficients);
    std::array<double, kMaxOeCoeff> getArcInclinationCoefficients(unsigned int arcNumber);
    void setArcArgPeriapsisCoefficients(unsigned int arcNumber,
                                        const std::array<double, kMaxOeCoeff>& argPeriapsisCoefficients);
    std::array<double, kMaxOeCoeff> getArcArgPeriapsisCoefficients(unsigned int arcNumber);
    void setArcRaanCoefficients(unsigned int arcNumber, const std::array<double, kMaxOeCoeff>& raanCoefficients);
    std::array<double, kMaxOeCoeff> getArcRaanCoefficients(unsigned int arcNumber);
    void setArcTrueAnomalyCoefficients(unsigned int arcNumber,
                                       const std::array<double, kMaxOeCoeff>& trueAnomalyCoefficients);
    std::array<double, kMaxOeCoeff> getArcTrueAnomalyCoefficients(unsigned int arcNumber);

   private:
    std::array<ChebyshevFitArc, kMaxOeRecords> fitCoefficients{};
    std::unique_ptr<OEStateEphemAlgorithm> algorithm = nullptr;
};

#endif
