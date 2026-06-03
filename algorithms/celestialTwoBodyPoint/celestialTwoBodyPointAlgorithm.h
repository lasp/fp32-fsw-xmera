#ifndef F32XMERA_CELESTIAL_BODY_POINT_ALGORITHM_H
#define F32XMERA_CELESTIAL_BODY_POINT_ALGORITHM_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include <Eigen/Core>

/*!@brief Algorithm that computes the two-body celestial pointing attitude reference.
 */
class CelestialTwoBodyPointAlgorithm final {
   public:
    /*! @brief Store whether the optional secondary celestial body input is available
        @param secCelBodyIsLinkedIn true if the secondary celestial body message is linked */
    void reset(bool secCelBodyIsLinkedIn);

    /*! @brief Compute the attitude reference that points at the primary celestial body while
        constraining a second axis toward the secondary celestial body when possible
        @param celBodyIn primary celestial body ephemeris
        @param secCelBodyIn secondary celestial body ephemeris (ignored when not linked)
        @param transNavIn spacecraft translational navigation solution
        @return attitude reference message payload */
    AttRefMsgF32Payload update(EphemerisMsgF32Payload &celBodyIn,
                               EphemerisMsgF32Payload &secCelBodyIn,
                               NavTransMsgF32Payload &transNavIn) const;

    /*! @brief Compute the reference attitude, angular velocity, and angular acceleration from the
        relative position and velocity of the primary and secondary celestial bodies
        @param r_PB_N [m] primary celestial body position relative to the spacecraft in inertial frame
        @param v_PB_N [m/s] primary celestial body velocity relative to the spacecraft in inertial frame
        @param r_SB_N [m] secondary celestial body position relative to the spacecraft in inertial frame
        @param v_SB_N [m/s] secondary celestial body velocity relative to the spacecraft in inertial frame
        @return attitude reference message payload */
    static AttRefMsgF32Payload rateAndAccelCalc(const Eigen::Vector3d &r_PB_N,
                                                const Eigen::Vector3d &v_PB_N,
                                                const Eigen::Vector3d &r_SB_N,
                                                const Eigen::Vector3d &v_SB_N);
    void setSingularityThreshold(float singularityThresholdIn);
    float getSingularityThreshold() const;
    void setRateThreshold(float rateThresholdIn);
    float getRateThreshold() const;

   private:
    float singularityThreshold{};  //!< [rad] Angle threshold below which the constraint axis is fixed
    float rateThreshold{};         //!< [rad/s] Rate threshold above which the constraint axis is fixed
    bool secCelBodyIsLinked{};     //!< Flag to indicate if the optional secondary celestial body message is linked
};

#endif
