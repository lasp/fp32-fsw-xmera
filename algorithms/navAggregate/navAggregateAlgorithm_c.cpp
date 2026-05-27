#include "navAggregateAlgorithm_c.h"
#include "navAggregateAlgorithm.h"

#include <Eigen/Core>
#include <array>

uint32_t NavAggregateAlgorithm_getMaxAggNavMsg(void) { return MAX_AGG_NAV_MSG; }

NavAggregateAlgorithmHandle* NavAggregateAlgorithm_create(void) {
    return reinterpret_cast<NavAggregateAlgorithmHandle*>(new ::f32::NavAggregateAlgorithm());
}

void NavAggregateAlgorithm_destroy(NavAggregateAlgorithmHandle* self) {
    delete reinterpret_cast<::f32::NavAggregateAlgorithm*>(self);
}

AggregateOutput_c NavAggregateAlgorithm_update(NavAggregateAlgorithmHandle* self,
                                               const NavAttMsgF32PayloadArray10_c* attMsgsPayloads,
                                               const NavTransMsgF32PayloadArray10_c* transMsgsPayloads) {
    /* Convert C payload arrays to Eigen-based internal types */
    std::array<f32::InputNavAttData, MAX_AGG_NAV_MSG> attArray{};
    std::array<f32::InputNavTransData, MAX_AGG_NAV_MSG> transArray{};

    for (uint32_t i = 0; i < MAX_AGG_NAV_MSG; ++i) {
        attArray[i].timeTag = attMsgsPayloads->msg[i].timeTag;
        attArray[i].sigma_BN = Eigen::Map<const Eigen::Vector3f>(attMsgsPayloads->msg[i].sigma_BN);
        attArray[i].omega_BN_B = Eigen::Map<const Eigen::Vector3f>(attMsgsPayloads->msg[i].omega_BN_B);
        attArray[i].vehSunPntBdy = Eigen::Map<const Eigen::Vector3f>(attMsgsPayloads->msg[i].vehSunPntBdy);

        transArray[i].timeTag = transMsgsPayloads->msg[i].timeTag;
        transArray[i].r_BN_N = Eigen::Map<const Eigen::Vector3d>(transMsgsPayloads->msg[i].r_BN_N);
        transArray[i].v_BN_N = Eigen::Map<const Eigen::Vector3d>(transMsgsPayloads->msg[i].v_BN_N);
        transArray[i].vehAccumDV = Eigen::Map<const Eigen::Vector3f>(transMsgsPayloads->msg[i].vehAccumDV);
    }

    f32::AggregateOutput result = reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->update(attArray, transArray);

    /* Convert Eigen-based output back to C-compatible POD types */
    AggregateOutput_c out{};

    out.navAttOut.timeTag = result.navAttOut.timeTag;
    Eigen::Map<Eigen::Vector3f>(out.navAttOut.sigma_BN) = result.navAttOut.sigma_BN;
    Eigen::Map<Eigen::Vector3f>(out.navAttOut.omega_BN_B) = result.navAttOut.omega_BN_B;
    Eigen::Map<Eigen::Vector3f>(out.navAttOut.vehSunPntBdy) = result.navAttOut.vehSunPntBdy;

    out.navTransOut.timeTag = result.navTransOut.timeTag;
    Eigen::Map<Eigen::Vector3d>(out.navTransOut.r_BN_N) = result.navTransOut.r_BN_N;
    Eigen::Map<Eigen::Vector3d>(out.navTransOut.v_BN_N) = result.navTransOut.v_BN_N;
    Eigen::Map<Eigen::Vector3f>(out.navTransOut.vehAccumDV) = result.navTransOut.vehAccumDV;

    return out;
}

void NavAggregateAlgorithm_setAttTimeIdx(NavAggregateAlgorithmHandle* self, uint32_t idx) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setAttTimeIdx(idx);
}

uint32_t NavAggregateAlgorithm_getAttTimeIdx(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getAttTimeIdx();
}

void NavAggregateAlgorithm_setTransTimeIdx(NavAggregateAlgorithmHandle* self, uint32_t idx) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setTransTimeIdx(idx);
}

uint32_t NavAggregateAlgorithm_getTransTimeIdx(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getTransTimeIdx();
}

void NavAggregateAlgorithm_setAttIdx(NavAggregateAlgorithmHandle* self, uint32_t idx) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setAttIdx(idx);
}

uint32_t NavAggregateAlgorithm_getAttIdx(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getAttIdx();
}

void NavAggregateAlgorithm_setRateIdx(NavAggregateAlgorithmHandle* self, uint32_t idx) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setRateIdx(idx);
}

uint32_t NavAggregateAlgorithm_getRateIdx(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getRateIdx();
}

void NavAggregateAlgorithm_setPosIdx(NavAggregateAlgorithmHandle* self, uint32_t idx) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setPosIdx(idx);
}

uint32_t NavAggregateAlgorithm_getPosIdx(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getPosIdx();
}

void NavAggregateAlgorithm_setVelIdx(NavAggregateAlgorithmHandle* self, uint32_t idx) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setVelIdx(idx);
}

uint32_t NavAggregateAlgorithm_getVelIdx(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getVelIdx();
}

void NavAggregateAlgorithm_setDvIdx(NavAggregateAlgorithmHandle* self, uint32_t idx) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setDvIdx(idx);
}

uint32_t NavAggregateAlgorithm_getDvIdx(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getDvIdx();
}

void NavAggregateAlgorithm_setSunIdx(NavAggregateAlgorithmHandle* self, uint32_t idx) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setSunIdx(idx);
}

uint32_t NavAggregateAlgorithm_getSunIdx(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getSunIdx();
}

void NavAggregateAlgorithm_setAttMsgCount(NavAggregateAlgorithmHandle* self, uint32_t msgCount) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setAttMsgCount(msgCount);
}

uint32_t NavAggregateAlgorithm_getAttMsgCount(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getAttMsgCount();
}

void NavAggregateAlgorithm_setTransMsgCount(NavAggregateAlgorithmHandle* self, uint32_t msgCount) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setTransMsgCount(msgCount);
}

uint32_t NavAggregateAlgorithm_getTransMsgCount(const NavAggregateAlgorithmHandle* self) {
    return reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->getTransMsgCount();
}
