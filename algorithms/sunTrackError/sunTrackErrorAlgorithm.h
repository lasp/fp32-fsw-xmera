#ifndef F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H
#define F32XMERA_SUN_TRACK_ERROR_ALGORITHM_H

#include <stdint.h>
#include <Eigen/Core>

struct SunTrackErrorNavAttInputs {
    Eigen::Vector3f sigma_BN{Eigen::Vector3f::Zero()};    //!< [-] measured MRP attitude of B wrt inertial N
    Eigen::Vector3f omega_BN_B{Eigen::Vector3f::Zero()};  //!< [r/s] measured body rate of B wrt N in B frame
};

struct SunTrackErrorAttRefInputs {
    Eigen::Vector3f sigma_RN{Eigen::Vector3f::Zero()};     //!< [-] reference MRP attitude of R wrt inertial N
    Eigen::Vector3f omega_RN_N{Eigen::Vector3f::Zero()};   //!< [r/s] reference rate of R wrt N in N frame
    Eigen::Vector3f domega_RN_N{Eigen::Vector3f::Zero()};  //!< [r/s^2] reference angular acceleration in N frame
};

struct SunTrackErrorOutput {
    Eigen::Vector3f sigma_BR{Eigen::Vector3f::Zero()};     //!< [-] attitude error MRP of B wrt R
    Eigen::Vector3f omega_BR_B{Eigen::Vector3f::Zero()};   //!< [r/s] body rate error of B wrt R in B frame
    Eigen::Vector3f omega_RN_B{Eigen::Vector3f::Zero()};   //!< [r/s] reference rate of R wrt N in B frame
    Eigen::Vector3f domega_RN_B{Eigen::Vector3f::Zero()};  //!< [r/s^2] reference angular acceleration in B frame
};

/*!@brief Module to compute the attitude tracking error for sun avoidance.
 */
class SunTrackErrorAlgorithm final {
   public:
    void reset(bool computeStartAngle);
    SunTrackErrorOutput update(const SunTrackErrorNavAttInputs& nav,
                               const SunTrackErrorAttRefInputs& ref,
                               const Eigen::Vector3f& r_BN_N,
                               const Eigen::Vector3f& r_SN_N,
                               uint64_t callTime);

    void setSigma_R0R(const Eigen::Vector3f& sigma);
    Eigen::Vector3f getSigma_R0R() const;
    void setSensitiveHat_B(const Eigen::Vector3f& sensitiveDirection);
    Eigen::Vector3f getSensitiveHat_B() const;
    void setAngleRate(float rate);
    float getAngleRate() const;

   private:
    SunTrackErrorOutput computeSunTrackError(const SunTrackErrorNavAttInputs& nav,
                                             const SunTrackErrorAttRefInputs& ref,
                                             uint64_t callTime) const;

    Eigen::Vector3f sigma_R0R{Eigen::Vector3f::Zero()}; /*!< MRP from corrected reference frame to original frame R0
                                                           This is the same as [BcB] going from primary body frame B
                                                           to the corrected body frame Bc */
    Eigen::Vector3f sensitiveHat_B{Eigen::Vector3f::Zero()};  //!< [-] Vehicle body vector to exclude from sun*/
    float angleRate{};                                        //!< [r/s] The rate at which we maneuver to Sun point*/
    Eigen::Vector3f mnvrAxis_B{Eigen::Vector3f::Zero()};      //!< [-] Eigen axis that we are maneuvering on*/
    float angleStart{};                                       //!< [r] The angle remaining in the attitude maneuver*/
    bool maneuverInitialized{};                               //!< [-] Flag indicating if maneuver has been set*/
    uint64_t mnvrStartTime{};                                 //!< [ns] Time at which the maneuver was begun*/
    bool computeAngleStart{};                                 /*!< [-] indicator whether angleStart should be computed
                                                               (if NavTransMsg and EphemerisMsg is linked)
                                                               or assumed to be 0 */
};

#endif
