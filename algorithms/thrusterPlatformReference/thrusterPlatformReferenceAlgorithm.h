#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H

#include <stdint.h>

#include <Eigen/Core>

inline constexpr int kMaxNumRw = 36;  //!< [-] maximum number of reaction wheels (must match RW_EFF_CNT)

/*! @brief Reaction-wheel spin-axis configuration used for momentum dumping. */
struct ThrusterPlatformReferenceRwArrayConfig {
    uint32_t numRW{};  //!< [-] number of reaction wheels on the vehicle
    Eigen::Matrix<float, 3, kMaxNumRw> GsMatrix_B{
        Eigen::Matrix<float, 3, kMaxNumRw>::Zero()};  //!< [-] RW spin axes in body frame, one column per wheel
    Eigen::Vector<float, kMaxNumRw> JsList{Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [kgm2] RW spin-axis inertias
};

/*! @brief Per-cycle inputs to the thruster platform reference algorithm. */
struct ThrusterPlatformReferenceInputs {
    Eigen::Vector3f r_CB_B{Eigen::Vector3f::Zero()};        //!< [m] center of mass w.r.t. B origin, B frame
    Eigen::Vector3f rThrust_F{Eigen::Vector3f::Zero()};     //!< [m] thrust application point w.r.t. F origin, F frame
    Eigen::Vector3f tHatThrust_F{Eigen::Vector3f::Zero()};  //!< [-] thrust unit direction, F frame
    float maxThrust{};                                      //!< [N] thrust magnitude
    Eigen::Vector<float, kMaxNumRw> wheelSpeeds{
        Eigen::Vector<float, kMaxNumRw>::Zero()};  //!< [r/s] reaction-wheel speeds
};

/*! @brief Outputs of the thruster platform reference algorithm. */
struct ThrusterPlatformReferenceOutput {
    float theta1{};                                              //!< [rad] platform tip reference angle
    float theta2{};                                              //!< [rad] platform tilt reference angle
    Eigen::Vector3f rHat_XB_B{Eigen::Vector3f::Zero()};          //!< [-] thrust heading unit vector, B frame
    Eigen::Vector3f torqueRequestBody{Eigen::Vector3f::Zero()};  //!< [Nm] torque to be compensated by the RWs, B frame
    Eigen::Vector3f rThrust_B{Eigen::Vector3f::Zero()};     //!< [m] thrust application point w.r.t. B origin, B frame
    Eigen::Vector3f tHatThrust_B{Eigen::Vector3f::Zero()};  //!< [-] thrust unit direction, B frame
    float maxThrust{};                                      //!< [N] thrust magnitude
};

/*! @brief Pure algorithm computing the tip/tilt reference of a dual-gimballed thruster platform. */
class ThrusterPlatformReferenceAlgorithm final {
   public:
    void reset(uint64_t callTime);
    ThrusterPlatformReferenceOutput update(const ThrusterPlatformReferenceInputs& in, uint64_t callTime);

    Eigen::Vector3f sigma_MB{Eigen::Vector3f::Zero()};  //!< [-] MRP orientation of the M frame w.r.t. the B frame
    Eigen::Vector3f r_BM_M{Eigen::Vector3f::Zero()};  //!< [m] B frame origin w.r.t. M frame origin, M frame coordinates
    Eigen::Vector3f r_FM_F{Eigen::Vector3f::Zero()};  //!< [m] F frame origin w.r.t. M frame origin, F frame coordinates
    float K{};                                        //!< [1/s] momentum dumping proportional gain
    float Ki{};                                       //!< [-] momentum dumping integral gain
    float theta1Max{};                                //!< [rad] absolute bound on the tip angle
    float theta2Max{};                                //!< [rad] absolute bound on the tilt angle
    bool momentumDumping{};                           //!< [-] whether reaction wheel momentum dumping is active
    ThrusterPlatformReferenceRwArrayConfig rwConfig{};  //!< [-] RW configuration used for momentum dumping

   private:
    Eigen::Vector3f hsInt_M{Eigen::Vector3f::Zero()};    //!< [Nms] integral of RW momentum, M frame
    Eigen::Vector3f priorHs_M{Eigen::Vector3f::Zero()};  //!< [Nms] prior RW momentum, M frame
    uint64_t priorTime{};                                //!< [ns] prior call time
};

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_H
