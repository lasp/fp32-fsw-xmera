#ifndef F32_XMERA_NAV_AGGREGATE_ALGORITHM_H
#define F32_XMERA_NAV_AGGREGATE_ALGORITHM_H

#include <stdint.h>

#include <Eigen/Core>
#include <array>

#define MAX_AGG_NAV_MSG 10U

namespace f32 {

/*! Struct containing the attitude navigation input needed by the algorithm. */
struct InputNavAttData {
    double timeTag{};
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f vehSunPntBdy = Eigen::Vector3f::Zero();
};

/*! Struct containing the translational navigation input needed by the algorithm. */
struct InputNavTransData {
    double timeTag{};
    Eigen::Vector3d r_BN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_BN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3f vehAccumDV = Eigen::Vector3f::Zero();
};

/*! structure containing the attitude and translational navigation outputs */
struct AggregateOutput {
    InputNavAttData navAttOut;     /*!< attitude navigation output */
    InputNavTransData navTransOut; /*!< translation navigation output */
};

class NavAggregateAlgorithm {
   public:
    AggregateOutput update(std::array<InputNavAttData, MAX_AGG_NAV_MSG> attInputs,
                           std::array<InputNavTransData, MAX_AGG_NAV_MSG> transInputs) const;
    void setAttTimeIdx(uint32_t idx);
    uint32_t getAttTimeIdx() const;
    void setTransTimeIdx(uint32_t idx);
    uint32_t getTransTimeIdx() const;
    void setAttIdx(uint32_t idx);
    uint32_t getAttIdx() const;
    void setRateIdx(uint32_t idx);
    uint32_t getRateIdx() const;
    void setPosIdx(uint32_t idx);
    uint32_t getPosIdx() const;
    void setVelIdx(uint32_t idx);
    uint32_t getVelIdx() const;
    void setDvIdx(uint32_t idx);
    uint32_t getDvIdx() const;
    void setSunIdx(uint32_t idx);
    uint32_t getSunIdx() const;
    void setAttMsgCount(uint32_t msgCount);
    uint32_t getAttMsgCount() const;
    void setTransMsgCount(uint32_t msgCount);
    uint32_t getTransMsgCount() const;

   private:
    uint32_t attTimeIdx{};    /*!< [-] The index of the message to use for attitude message time */
    uint32_t transTimeIdx{};  /*!< [-] The index of the message to use for translation message time */
    uint32_t attIdx{};        /*!< [-] The index of the message to use for inertial MRP*/
    uint32_t rateIdx{};       /*!< [-] The index of the message to use for attitude rate*/
    uint32_t posIdx{};        /*!< [-] The index of the message to use for inertial position*/
    uint32_t velIdx{};        /*!< [-] The index of the message to use for inertial velocity*/
    uint32_t dvIdx{};         /*!< [-] The index of the message to use for accumulated DV */
    uint32_t sunIdx{};        /*!< [-] The index of the message to use for sun pointing*/
    uint32_t attMsgCount{};   /*!< [-] The total number of messages available as inputs */
    uint32_t transMsgCount{}; /*!< [-] The total number of messages available as inputs */
};

}  // namespace f32

#endif
