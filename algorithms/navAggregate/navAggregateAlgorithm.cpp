#include "navAggregateAlgorithm.h"

#include <array>

using namespace f32;

/*! @brief Construct the algorithm with a validated configuration. */
NavAggregateAlgorithm::NavAggregateAlgorithm(const NavAggregateConfig& config) : cfg(config) { setConfig(config); }

/*! @brief Replace the algorithm's stored configuration at runtime. */
void NavAggregateAlgorithm::setConfig(const NavAggregateConfig& config) { this->cfg = config; }

/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return AggregateOutput attitude navigation and translation navigation output
 @param attInputs Aggregated attitude navigation inputs
 @param transInputs Aggregated translational navigation inputs
 */
AggregateOutput NavAggregateAlgorithm::update(std::array<InputNavAttData, MAX_AGG_NAV_MSG> attInputs,
                                              std::array<InputNavTransData, MAX_AGG_NAV_MSG> transInputs) const {
    const NavAggregateAttSelection& att = this->cfg.getAttSelection();
    const NavAggregateTransSelection& trans = this->cfg.getTransSelection();

    InputNavAttData navAttOutput{};     /* [-] local storage of the outgoing attitude navigation data*/
    InputNavTransData navTransOutput{}; /* [-] local storage of the outgoing translation navigation data*/

    /*! - check that attitude navigation messages are present */
    if (att.attMsgCount > 0U) {
        /*! - Copy out each part of the attitude source message into the target output message*/
        navAttOutput.timeTag = attInputs.at(att.attTimeIdx).timeTag;
        navAttOutput.sigma_BN = attInputs.at(att.attIdx).sigma_BN;
        navAttOutput.omega_BN_B = attInputs.at(att.rateIdx).omega_BN_B;
        navAttOutput.vehSunPntBdy = attInputs.at(att.sunIdx).vehSunPntBdy;
    }

    /*! - check that translation navigation messages are present */
    if (trans.transMsgCount > 0U) {
        /*! - Copy out each part of the translation source message into the target output message*/
        navTransOutput.timeTag = transInputs.at(trans.transTimeIdx).timeTag;
        navTransOutput.r_BN_N = transInputs.at(trans.posIdx).r_BN_N;
        navTransOutput.v_BN_N = transInputs.at(trans.velIdx).v_BN_N;
        navTransOutput.vehAccumDV = transInputs.at(trans.dvIdx).vehAccumDV;
    }

    AggregateOutput navAggregateOut{};
    navAggregateOut.navAttOut = navAttOutput;
    navAggregateOut.navTransOut = navTransOutput;

    return navAggregateOut;
}
