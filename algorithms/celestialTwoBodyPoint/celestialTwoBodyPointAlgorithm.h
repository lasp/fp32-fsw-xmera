#ifndef F32XMERA_CELESTIAL_BODY_POINT_ALGORITHM_H
#define F32XMERA_CELESTIAL_BODY_POINT_ALGORITHM_H

#include "celestialTwoBodyPointTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <Eigen/Core>

/*!@brief Output of the two-body celestial pointing algorithm.
 */
struct CelestialTwoBodyPointOutput {
    Eigen::Vector3f sigma_RN = Eigen::Vector3f::Zero();     //!< MRP attitude of reference frame relative to inertial
    Eigen::Vector3f omega_RN_N = Eigen::Vector3f::Zero();   //!< [rad/s] Reference frame angular velocity
    Eigen::Vector3f domega_RN_N = Eigen::Vector3f::Zero();  //!< [rad/s^2] Reference frame angular acceleration
};

/*!@brief Validated configuration for the two-body celestial pointing algorithm.
 */
class CelestialTwoBodyPointConfig final {
   public:
    /*! @brief Static factory — validates all parameters, throws on failure
        @param singularityThreshold [rad] angle threshold below which the constraint axis is fixed
        @param rateThreshold [rad/s] rate threshold above which the constraint axis is fixed
        @param secCelBodyIsLinked true if the secondary celestial body message is linked
        @return validated configuration object */
    static CelestialTwoBodyPointConfig create(float singularityThreshold,
                                              float rateThreshold,
                                              bool secCelBodyIsLinked) {
        if (!isValidSingularityThreshold(singularityThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("celestialTwoBodyPoint: singularityThreshold must be >= 0");
        }
        if (!isValidRateThreshold(rateThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("celestialTwoBodyPoint: rateThreshold must be >= 0");
        }
        return {singularityThreshold, rateThreshold, secCelBodyIsLinked};
    }

    static bool isValidSingularityThreshold(float singularityThreshold) { return singularityThreshold >= 0.0F; }
    static bool isValidRateThreshold(float rateThreshold) { return rateThreshold >= 0.0F; }
    // No isValidSecCelBodyIsLinked — bool with no semantic constraint, validator would be vacuous.

    float getSingularityThreshold() const { return singularityThreshold; }
    float getRateThreshold() const { return rateThreshold; }
    bool getSecCelBodyIsLinked() const { return secCelBodyIsLinked; }

   private:
    CelestialTwoBodyPointConfig(float singularityThreshold, float rateThreshold, bool secCelBodyIsLinked)
        : singularityThreshold(singularityThreshold),
          rateThreshold(rateThreshold),
          secCelBodyIsLinked(secCelBodyIsLinked) {}

    float singularityThreshold;  //!< [rad] Angle threshold below which the constraint axis is fixed
    float rateThreshold;         //!< [rad/s] Rate threshold above which the constraint axis is fixed
    bool secCelBodyIsLinked;     //!< Flag to indicate if the optional secondary celestial body message is linked
};

/*!@brief Algorithm that computes the two-body celestial pointing attitude reference.
 */
class CelestialTwoBodyPointAlgorithm final {
   public:
    /*! @brief Construct the algorithm from a validated configuration
        @param config validated configuration object */
    explicit CelestialTwoBodyPointAlgorithm(const CelestialTwoBodyPointConfig &config);

    /*! @brief Replace the algorithm configuration
        @param config validated configuration object */
    void setConfig(const CelestialTwoBodyPointConfig &config);

    /*! @brief Compute the attitude reference that points at the primary celestial body while
        constraining a second axis toward the secondary celestial body when possible
        @param r_celBody_N [m] primary celestial body inertial position
        @param v_celBody_N [m/s] primary celestial body inertial velocity
        @param r_secCelBody_N [m] secondary celestial body inertial position (ignored when not linked)
        @param v_secCelBody_N [m/s] secondary celestial body inertial velocity (ignored when not linked)
        @param r_BN_N [m] spacecraft inertial position
        @param v_BN_N [m/s] spacecraft inertial velocity
        @return attitude reference output */
    CelestialTwoBodyPointOutput update(const Eigen::Vector3d &r_celBody_N,
                                       const Eigen::Vector3d &v_celBody_N,
                                       const Eigen::Vector3d &r_secCelBody_N,
                                       const Eigen::Vector3d &v_secCelBody_N,
                                       const Eigen::Vector3d &r_BN_N,
                                       const Eigen::Vector3d &v_BN_N) const;

    /*! @brief Compute the reference attitude, angular velocity, and angular acceleration from the
        relative position and velocity of the primary and secondary celestial bodies
        @param r_PB_N [m] primary celestial body position relative to the spacecraft in inertial frame
        @param v_PB_N [m/s] primary celestial body velocity relative to the spacecraft in inertial frame
        @param r_SB_N [m] secondary celestial body position relative to the spacecraft in inertial frame
        @param v_SB_N [m/s] secondary celestial body velocity relative to the spacecraft in inertial frame
        @return attitude reference output */
    static CelestialTwoBodyPointOutput rateAndAccelCalc(const Eigen::Vector3d &r_PB_N,
                                                        const Eigen::Vector3d &v_PB_N,
                                                        const Eigen::Vector3d &r_SB_N,
                                                        const Eigen::Vector3d &v_SB_N);

   private:
    CelestialTwoBodyPointConfig cfg;  //!< Validated algorithm configuration
};

#endif
