/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_OE_STATE_EPHEM_H
#define F32XIMERA_OE_STATE_EPHEM_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/TDBVehicleClockCorrelationMsgF32Payload.h"
#include "oeStateEphemAlgorithm.h"

#define MAX_OE_RECORDS 10
#define MAX_OE_COEFF 20

/*! @brief Top level structure for the Chebyshev position ephemeris
           fit system.  Allows the user to specify a set of chebyshev
           coefficients and then use the input time to determine where
           a given body is in space
*/
class OEStateEphem : public SysModel {
   public:
    OEStateEphem() = default;
    ~OEStateEphem() final = default;

    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    Message<EphemerisMsgF32Payload> stateFitOutMsg;                       //!< [-] output navigation message for pos/vel
    ReadFunctor<TDBVehicleClockCorrelationMsgF32Payload> clockCorrInMsg;  //!< clock correlation input message

    void setCentralBodyGravitationalParameter(float gravitationalParameter);
    float getCentralBodyGravitationalParameter() const;

    void setArcNumberOfCoefficients(unsigned int arcNumber, unsigned int numberOfCoefficients);
    unsigned int getArcNumberOfCoefficients(unsigned int arcNumber) const;

    void setArcMiddleTime(unsigned int arcNumber, double timeMiddle);
    double getArcMiddleTime(unsigned int arcNumber) const;

    void setArcRadiusTime(unsigned int arcNumber, double timeRadius);
    double getArcRadiusTime(unsigned int arcNumber) const;

    void setArcAnomalyFlag(unsigned int arcNumber, unsigned int anomalyFlag);
    unsigned int getArcAnomalyFlag(unsigned int arcNumber) const;

    void setArcRadiusPeriapsisCoefficients(unsigned int arcNumber,
                                           const std::array<double, MAX_OE_COEFF>& radiusPeriapsisCoefficients);
    std::array<double, MAX_OE_COEFF> getArcRadiusPeriapsisCoefficients(unsigned int arcNumber);
    void setArcEccentricityCoefficients(unsigned int arcNumber,
                                        const std::array<float, MAX_OE_COEFF> &eccentricityCoefficients);
    std::array<float, MAX_OE_COEFF> getArcEccentricityCoefficients(unsigned int arcNumber);
    void setArcInclinationCoefficients(unsigned int arcNumber,
                                       const std::array<float, MAX_OE_COEFF> &inclinationCoefficients);
    std::array<float, MAX_OE_COEFF> getArcInclinationCoefficients(unsigned int arcNumber);
    void setArcArgPeriapsisCoefficients(unsigned int arcNumber,
                                        const std::array<float, MAX_OE_COEFF> &argPeriapsisCoefficients);
    std::array<float, MAX_OE_COEFF> getArcArgPeriapsisCoefficients(unsigned int arcNumber);
    void setArcRaanCoefficients(unsigned int arcNumber, const std::array<float, MAX_OE_COEFF> &raanCoefficients);
    std::array<float, MAX_OE_COEFF> getArcRaanCoefficients(unsigned int arcNumber);
    void setArcTrueAnomalyCoefficients(unsigned int arcNumber,
                                       const std::array<float, MAX_OE_COEFF> &trueAnomalyCoefficients);
    std::array<float, MAX_OE_COEFF> getArcTrueAnomalyCoefficients(unsigned int arcNumber);

   private:
    OEStateEphemAlgorithm algorithm{};
};

#endif
