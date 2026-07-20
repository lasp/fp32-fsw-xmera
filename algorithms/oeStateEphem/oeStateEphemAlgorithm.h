#ifndef F32XIMERA_OE_STATE_EPHEM_ALGORITHM_H
#define F32XIMERA_OE_STATE_EPHEM_ALGORITHM_H

#include "oeStateEphemTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/orbitalMotion.hpp"
#include <array>
#include <cstddef>

inline constexpr double kTolerance = 1e-10;
inline constexpr std::size_t kMaxOeCoeff = MAX_OE_COEFF;
inline constexpr std::size_t kMaxOeRecords = MAX_OE_RECORDS;

/*! @brief Structure that defines the layout of an Ephemeris "record."  This is
           basically the set of coefficients for the ephemeris elements and
           the time factors associated with those coefficients
*/
struct ChebyshevFitArc {
    unsigned int numberChebCoefficients{};  //!< [-] Number chebyshev coefficients loaded into record
    double ephemerisTimeMiddle{};           //!< [s] Ephemeris time (TDB) associated with the mid-point of the curve
    double ephemerisTimeRadius{};           //!< [s] "Radius" of time that curve is valid for (half of total range)
    std::array<double, kMaxOeCoeff>
        radiusPeriapsisCoefficients{};  //!< [-] Set of chebyshev coefficients for radius at periapses
    std::array<double, kMaxOeCoeff> eccentricityCoefficients{};  //!< [-] Set of chebyshev coefficients for eccentricity
    std::array<double, kMaxOeCoeff> inclinationCoefficients{};   //!< [-] Set of chebyshev coefficients for inclination
    std::array<double, kMaxOeCoeff>
        argPeriapsisCoefficients{};  //!< [-] Set of chebyshev coefficients for argument of periapses
    std::array<double, kMaxOeCoeff>
        raanCoefficients{};  //!< [-] Set of chebyshev coefficients for right ascention of the ascending node
    std::array<double, kMaxOeCoeff>
        trueAnomalyCoefficients{};                       //!< [-] Set of chebyshev coefficients for true anomaly angle
    AnomalyType anomalyFlag{AnomalyType::TRUE_ANOMALY};  //!< [-] Flag indicating if the anomaly angle is true or mean
};

/*!
 * @brief Validated configuration for the OE state ephemeris algorithm.
 *
 * Holds the central-body gravitational parameter, the active arc count, the constant ephemeris/vehicle time
 * offsets, and the table of Chebyshev fit arcs. An instance can only exist via OEStateEphemConfig::create(...),
 * which enforces: mu >= 0; 1 <= numberOfArcs <= kMaxOeRecords; ephemerisTimeJ2000 >= 0; vehicleTimeOffset >= 0;
 * and for every active arc, numberChebCoefficients >= 1 with positive ephemerisTimeMiddle and ephemerisTimeRadius.
 */
class OEStateEphemConfig final {
   public:
    static OEStateEphemConfig create(double centralBodyGravitationalParameter,
                                     unsigned int numberOfArcs,
                                     double ephemerisTimeJ2000,
                                     double vehicleTimeOffset,
                                     const std::array<ChebyshevFitArc, kMaxOeRecords>& fitCoefficients) {
        if (!isValidGravitationalParameter(centralBodyGravitationalParameter)) {
            FSW_THROW_INVALID_ARGUMENT("OEStateEphem: gravitational parameter must be non-negative.");
        }
        if (!isValidNumberOfArcs(numberOfArcs)) {
            FSW_THROW_INVALID_ARGUMENT("OEStateEphem: numberOfArcs must be in [1, kMaxOeRecords].");
        }
        if (!isValidTimeOffset(ephemerisTimeJ2000)) {
            FSW_THROW_INVALID_ARGUMENT("OEStateEphem: ephemeris J2000 time must be non-negative.");
        }
        if (!isValidTimeOffset(vehicleTimeOffset)) {
            FSW_THROW_INVALID_ARGUMENT("OEStateEphem: vehicle time offset must be non-negative.");
        }
        for (unsigned int i = 0U; i < numberOfArcs; ++i) {
            if (!isValidArc(fitCoefficients.at(i))) {
                FSW_THROW_INVALID_ARGUMENT(
                    "OEStateEphem: each active arc needs numberChebCoefficients >= 1 and positive middle/radius time.");
            }
        }
        return {
            centralBodyGravitationalParameter, numberOfArcs, ephemerisTimeJ2000, vehicleTimeOffset, fitCoefficients};
    }

    static bool isValidGravitationalParameter(double gravitationalParameter) { return gravitationalParameter >= 0.0; }
    static bool isValidNumberOfArcs(unsigned int numberOfArcs) {
        return numberOfArcs >= 1U && numberOfArcs <= kMaxOeRecords;
    }
    static bool isValidTimeOffset(double timeOffset) { return timeOffset >= 0.0; }
    static bool isValidArc(const ChebyshevFitArc& arc) {
        return arc.numberChebCoefficients >= 1U && arc.ephemerisTimeMiddle > 0.0 && arc.ephemerisTimeRadius > 0.0;
    }

    double getCentralBodyGravitationalParameter() const { return centralBodyGravitationalParameter; }
    unsigned int getNumberOfArcs() const { return numberOfArcs; }
    double getEphemerisTimeJ2000() const { return ephemerisTimeJ2000; }
    double getVehicleTimeOffset() const { return vehicleTimeOffset; }
    const std::array<ChebyshevFitArc, kMaxOeRecords>& getFitCoefficients() const { return fitCoefficients; }

   private:
    // The config fields have distinct meanings but several share a type; construction is funneled through the
    // named create() factory, which makes the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    OEStateEphemConfig(double centralBodyGravitationalParameter,
                       unsigned int numberOfArcs,
                       double ephemerisTimeJ2000,
                       double vehicleTimeOffset,
                       const std::array<ChebyshevFitArc, kMaxOeRecords>& fitCoefficients)
        : centralBodyGravitationalParameter(centralBodyGravitationalParameter),
          numberOfArcs(numberOfArcs),
          ephemerisTimeJ2000(ephemerisTimeJ2000),
          vehicleTimeOffset(vehicleTimeOffset),
          fitCoefficients(fitCoefficients) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    double centralBodyGravitationalParameter;
    unsigned int numberOfArcs;
    double ephemerisTimeJ2000;
    double vehicleTimeOffset;
    std::array<ChebyshevFitArc, kMaxOeRecords> fitCoefficients;
};

/*! @brief Top level structure for the Chebyshev position ephemeris fit system. Evaluates the configured
           Chebyshev coefficients at the requested time to determine where a given body is in space.
*/
class OEStateEphemAlgorithm final {
   public:
    explicit OEStateEphemAlgorithm(const OEStateEphemConfig& config);

    void setConfig(const OEStateEphemConfig& config);

    orbitalMotion::CartesianState update(uint64_t callTime) const;

   private:
    ChebyshevFitArc findCurrentArc(double currentEphTime) const;
    static double scaleEphemerisTime(const ChebyshevFitArc& arc, double currentEphTime);
    static orbitalMotion::ClassicalElements evaluateCoefficients(double currentScaledValue, const ChebyshevFitArc& arc);
    bool allParametersNull() const;

    OEStateEphemConfig cfg;
};

#endif
