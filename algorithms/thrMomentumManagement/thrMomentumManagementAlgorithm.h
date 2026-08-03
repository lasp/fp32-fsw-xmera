#ifndef F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_H
#define F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_H

#include "thrMomentumManagementTypes.h"
#include <stdint.h>

#include <Eigen/Core>
#include <optional>

inline constexpr uint32_t kMaxNumRw = THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW;  //!< [-] maximum number of reaction wheels

/*! @brief Reaction-wheel spin-axis configuration used to compute the net cluster momentum. */
struct ThrMomentumManagementRwArrayConfiguration {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    Eigen::Vector<float, kMaxNumRw> JsList{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [kgm2] RW spin-axis inertias
};

/*!
 * @brief Assesses the net reaction wheel momentum and computes the angular momentum change needed to dump it.
 *
 * The momentum check is a one-shot: it runs on the first update() after a re-initialization and is then
 * disarmed, so a dump is requested at most once per re-initialization. Call reInitialize() to re-arm it.
 */
class ThrMomentumManagementAlgorithm final {
   public:
    //! Re-arm the one-shot momentum dumping request.
    void reInitialize();

    //! [Nms] Requested body-frame angular momentum change, or nullopt once the one-shot check has been spent.
    std::optional<Eigen::Vector3f> update(const Eigen::Vector<float, kMaxNumRw>& wheelSpeeds);

    float hs_min{};                                           //!< [Nms] minimum RW cluster momentum for dumping
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;  //!< [-] RW spin axes and spin-axis inertias

   private:
    bool dumpRequested{};  //!< [-] true while the one-shot momentum check is still pending
};

#endif
