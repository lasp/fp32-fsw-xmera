/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_OE_STATE_EPHEM_H
#define F32XIMERA_OE_STATE_EPHEM_H

#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/TDBVehicleClockCorrelationMsgF32Payload.h"
#include "oeStateEphemAlgorithm.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

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

    Message<EphemerisMsgF32Payload> stateFitOutMsg;                       //!< [-] output navigation message for pos/vel
    ReadFunctor<TDBVehicleClockCorrelationMsgF32Payload> clockCorrInMsg;  //!< clock correlation input message

    void setCentralBodyGravitationalParameter(float gravitationalParameter);
    float getCentralBodyGravitationalParameter() const;

    void setSpiceBodyId(int spiceId);
    int getSpiceBodyId() const;

    void setCentralBodyId(int centralBody);
    int getCentralBodyId() const;

    void setArcNumberOfCoefficients(unsigned int arcNumber, unsigned int numberOfCoefficients);
    unsigned int getArcNumberOfCoefficients(unsigned int arcNumber) const;

    void setArcMiddleTime(unsigned int arcNumber, double timeMiddle);
    double getArcMiddleTime(unsigned int arcNumber) const;

    void setArcRadiusTime(unsigned int arcNumber, double timeRadius);
    double getArcRadiusTime(unsigned int arcNumber) const;

    void setArcAnomalyFlag(unsigned int arcNumber, const AnomalyType &anomalyFlag);
    AnomalyType getArcAnomalyFlag(unsigned int arcNumber) const;

    void setArcRadiusPeriapsisCoefficients(unsigned int arcNumber,
                                           const std::array<double, kMaxOeCoeff> &radiusPeriapsisCoefficients);
    std::array<double, kMaxOeCoeff> getArcRadiusPeriapsisCoefficients(unsigned int arcNumber);
    void setArcEccentricityCoefficients(unsigned int arcNumber,
                                        const std::array<float, kMaxOeCoeff> &eccentricityCoefficients);
    std::array<float, kMaxOeCoeff> getArcEccentricityCoefficients(unsigned int arcNumber);
    void setArcInclinationCoefficients(unsigned int arcNumber,
                                       const std::array<float, kMaxOeCoeff> &inclinationCoefficients);
    std::array<float, kMaxOeCoeff> getArcInclinationCoefficients(unsigned int arcNumber);
    void setArcArgPeriapsisCoefficients(unsigned int arcNumber,
                                        const std::array<float, kMaxOeCoeff> &argPeriapsisCoefficients);
    std::array<float, kMaxOeCoeff> getArcArgPeriapsisCoefficients(unsigned int arcNumber);
    void setArcRaanCoefficients(unsigned int arcNumber, const std::array<float, kMaxOeCoeff> &raanCoefficients);
    std::array<float, kMaxOeCoeff> getArcRaanCoefficients(unsigned int arcNumber);
    void setArcTrueAnomalyCoefficients(unsigned int arcNumber,
                                       const std::array<float, kMaxOeCoeff> &trueAnomalyCoefficients);
    std::array<float, kMaxOeCoeff> getArcTrueAnomalyCoefficients(unsigned int arcNumber);

   private:
    OEStateEphemAlgorithm algorithm{};
    int spiceBodyId{};
    int centralBodyId{};
};

#endif
