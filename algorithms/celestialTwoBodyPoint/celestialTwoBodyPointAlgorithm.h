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
        @param const celestialBodyAlignmentThreshold [rad] Angle threshold for primary and secondary celestial body
       alignment check
        @return validated configuration object */
    static CelestialTwoBodyPointConfig create(const float celestialBodyAlignmentThreshold) {
        if (!isValidCelestialBodyAlignmentThreshold(celestialBodyAlignmentThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("celestialTwoBodyPoint: celestialBodyAlignmentThreshold must be >= 1e-6");
        }
        return {celestialBodyAlignmentThreshold};
    }

    static bool isValidCelestialBodyAlignmentThreshold(const float celestialBodyAlignmentThreshold) {
        return celestialBodyAlignmentThreshold >= 1e-6F;
    }
    float getCelestialBodyAlignmentThreshold() const { return celestialBodyAlignmentThreshold; }

   private:
    CelestialTwoBodyPointConfig(float celestialBodyAlignmentThreshold)
        : celestialBodyAlignmentThreshold(celestialBodyAlignmentThreshold) {}

    float celestialBodyAlignmentThreshold{};  //!< [rad] Angle threshold for primary and secondary celestial body
                                              //!< alignment check
};

/*!@brief Algorithm that computes the two-body celestial pointing attitude reference.
 */
class CelestialTwoBodyPointAlgorithm final {
   public:
    /// @c |r_SB_N|^2 and |r_PB_N|^2 floor: below this the configuration is invalid.
    static constexpr float kMinNormSq = 1e-6F;

    /*! @brief Construct the algorithm from a validated configuration
        @param config validated configuration object */
    explicit CelestialTwoBodyPointAlgorithm(const CelestialTwoBodyPointConfig &config);

    /*! @brief Replace the algorithm configuration
        @param config validated configuration object */
    void setConfig(const CelestialTwoBodyPointConfig &config);

    /*! @brief Compute the attitude reference that points at the primary celestial body while
        constraining a second axis toward the secondary celestial body when possible
        @param r_PN_N [m] primary celestial body inertial position
        @param v_PN_N [m/s] primary celestial body inertial velocity
        @param r_SN_N [m] secondary celestial body inertial position
        @param v_SN_N [m/s] secondary celestial body inertial velocity
        @param r_BN_N [m] spacecraft inertial position
        @param v_BN_N [m/s] spacecraft inertial velocity
        @return attitude reference output */
    CelestialTwoBodyPointOutput update(const Eigen::Vector3d &r_PN_N,
                                       const Eigen::Vector3d &v_PN_N,
                                       const Eigen::Vector3d &r_SN_N,
                                       const Eigen::Vector3d &v_SN_N,
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
