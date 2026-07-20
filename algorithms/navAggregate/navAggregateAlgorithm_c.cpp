#include "navAggregateAlgorithm_c.h"
#include "navAggregateAlgorithm.h"
#include "navAggregateTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <array>

// The C-boundary aggregate message count must match the algorithm's MAX_AGG_NAV_MSG, otherwise the fixed-size
// C payload arrays and the std::array conversions below would disagree on the message count.
static_assert(MAX_AGG_NAV_MSG_C == MAX_AGG_NAV_MSG, "MAX_AGG_NAV_MSG_C must match MAX_AGG_NAV_MSG");

namespace {
f32::NavAggregateConfig configFromC(const NavAggregateConfig_c& c) {
    const f32::NavAggregateAttSelection attSelection{
        .attTimeIdx = c.attSelection.attTimeIdx,
        .attIdx = c.attSelection.attIdx,
        .rateIdx = c.attSelection.rateIdx,
        .sunIdx = c.attSelection.sunIdx,
        .attMsgCount = c.attSelection.attMsgCount,
    };
    const f32::NavAggregateTransSelection transSelection{
        .transTimeIdx = c.transSelection.transTimeIdx,
        .posIdx = c.transSelection.posIdx,
        .velIdx = c.transSelection.velIdx,
        .dvIdx = c.transSelection.dvIdx,
        .transMsgCount = c.transSelection.transMsgCount,
    };
    return f32::NavAggregateConfig::create(attSelection, transSelection);
}
}  // namespace

uint32_t NavAggregateAlgorithm_getMaxAggNavMsg(void) { return MAX_AGG_NAV_MSG; }

NavAggregateAlgorithmHandle* NavAggregateAlgorithm_create(const NavAggregateConfig_c* config) {
    return reinterpret_cast<NavAggregateAlgorithmHandle*>(new ::f32::NavAggregateAlgorithm(configFromC(*config)));
}

void NavAggregateAlgorithm_destroy(NavAggregateAlgorithmHandle* self) {
    delete reinterpret_cast<::f32::NavAggregateAlgorithm*>(self);
}

void NavAggregateAlgorithm_setConfig(NavAggregateAlgorithmHandle* self, const NavAggregateConfig_c* config) {
    reinterpret_cast<::f32::NavAggregateAlgorithm*>(self)->setConfig(configFromC(*config));
}

AggregateOutput_c NavAggregateAlgorithm_update(const NavAggregateAlgorithmHandle* self,
                                               const NavAttMsgF32PayloadArray10_c* attMsgsPayloads,
                                               const NavTransMsgF32PayloadArray10_c* transMsgsPayloads) {
    /* Convert C payload arrays to Eigen-based internal types */
    std::array<f32::InputNavAttData, MAX_AGG_NAV_MSG> attArray{};
    std::array<f32::InputNavTransData, MAX_AGG_NAV_MSG> transArray{};

    for (uint32_t i = 0U; i < MAX_AGG_NAV_MSG; ++i) {
        attArray[i].timeTag = attMsgsPayloads->msg[i].timeTag;
        attArray[i].sigma_BN = cArrayToEigenVector(attMsgsPayloads->msg[i].sigma_BN);
        attArray[i].omega_BN_B = cArrayToEigenVector(attMsgsPayloads->msg[i].omega_BN_B);
        attArray[i].vehSunPntBdy = cArrayToEigenVector(attMsgsPayloads->msg[i].vehSunPntBdy);

        transArray[i].timeTag = transMsgsPayloads->msg[i].timeTag;
        transArray[i].r_BN_N = cArrayToEigenVector(transMsgsPayloads->msg[i].r_BN_N);
        transArray[i].v_BN_N = cArrayToEigenVector(transMsgsPayloads->msg[i].v_BN_N);
        transArray[i].vehAccumDV = cArrayToEigenVector(transMsgsPayloads->msg[i].vehAccumDV);
    }

    const f32::AggregateOutput result =
        reinterpret_cast<const ::f32::NavAggregateAlgorithm*>(self)->update(attArray, transArray);

    /* Convert Eigen-based output back to C-compatible POD types */
    AggregateOutput_c out{};

    out.navAttOut.timeTag = result.navAttOut.timeTag;
    eigenVectorToCArray(result.navAttOut.sigma_BN, out.navAttOut.sigma_BN);
    eigenVectorToCArray(result.navAttOut.omega_BN_B, out.navAttOut.omega_BN_B);
    eigenVectorToCArray(result.navAttOut.vehSunPntBdy, out.navAttOut.vehSunPntBdy);

    out.navTransOut.timeTag = result.navTransOut.timeTag;
    eigenVectorToCArray(result.navTransOut.r_BN_N, out.navTransOut.r_BN_N);
    eigenVectorToCArray(result.navTransOut.v_BN_N, out.navTransOut.v_BN_N);
    eigenVectorToCArray(result.navTransOut.vehAccumDV, out.navTransOut.vehAccumDV);

    return out;
}
