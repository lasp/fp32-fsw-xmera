#ifndef F32_XMERA_NAV_AGGREGATE_ALGORITHM_H
#define F32_XMERA_NAV_AGGREGATE_ALGORITHM_H

#include <stdint.h>

#include "utilities/fsw/freestandingInvalidArgument.h"
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

/*! Message-selection indices for the attitude navigation aggregation. */
struct NavAggregateAttSelection {
    uint32_t attTimeIdx{};   //!< [-] index of the message providing the attitude message time
    uint32_t attIdx{};       //!< [-] index of the message providing the inertial MRP
    uint32_t rateIdx{};      //!< [-] index of the message providing the attitude rate
    uint32_t sunIdx{};       //!< [-] index of the message providing the sun-pointing vector
    uint32_t attMsgCount{};  //!< [-] number of attitude messages available as inputs
};

/*! Message-selection indices for the translational navigation aggregation. */
struct NavAggregateTransSelection {
    uint32_t transTimeIdx{};   //!< [-] index of the message providing the translation message time
    uint32_t posIdx{};         //!< [-] index of the message providing the inertial position
    uint32_t velIdx{};         //!< [-] index of the message providing the inertial velocity
    uint32_t dvIdx{};          //!< [-] index of the message providing the accumulated DV
    uint32_t transMsgCount{};  //!< [-] number of translation messages available as inputs
};

/*!
 * @brief Validated configuration for the navigation aggregation algorithm.
 *
 * An instance can only exist when every selection index is less than MAX_AGG_NAV_MSG and each message count
 * does not exceed MAX_AGG_NAV_MSG. Construct via NavAggregateConfig::create(...).
 */
class NavAggregateConfig final {
   public:
    static NavAggregateConfig create(const NavAggregateAttSelection& attSelection,
                                     const NavAggregateTransSelection& transSelection) {
        if (!isValidAttSelection(attSelection)) {
            FSW_THROW_INVALID_ARGUMENT(
                "navAggregate: attitude selection indices must be < MAX_AGG_NAV_MSG and attMsgCount must be "
                "<= MAX_AGG_NAV_MSG.");
        }
        if (!isValidTransSelection(transSelection)) {
            FSW_THROW_INVALID_ARGUMENT(
                "navAggregate: translation selection indices must be < MAX_AGG_NAV_MSG and transMsgCount must be "
                "<= MAX_AGG_NAV_MSG.");
        }
        return {attSelection, transSelection};
    }

    static bool isValidAttSelection(const NavAggregateAttSelection& attSelection) {
        return attSelection.attTimeIdx < MAX_AGG_NAV_MSG && attSelection.attIdx < MAX_AGG_NAV_MSG &&
               attSelection.rateIdx < MAX_AGG_NAV_MSG && attSelection.sunIdx < MAX_AGG_NAV_MSG &&
               attSelection.attMsgCount <= MAX_AGG_NAV_MSG;
    }

    static bool isValidTransSelection(const NavAggregateTransSelection& transSelection) {
        return transSelection.transTimeIdx < MAX_AGG_NAV_MSG && transSelection.posIdx < MAX_AGG_NAV_MSG &&
               transSelection.velIdx < MAX_AGG_NAV_MSG && transSelection.dvIdx < MAX_AGG_NAV_MSG &&
               transSelection.transMsgCount <= MAX_AGG_NAV_MSG;
    }

    const NavAggregateAttSelection& getAttSelection() const { return this->attSelection; }
    const NavAggregateTransSelection& getTransSelection() const { return this->transSelection; }

   private:
    NavAggregateConfig(const NavAggregateAttSelection& attSelection, const NavAggregateTransSelection& transSelection)
        : attSelection(attSelection), transSelection(transSelection) {}

    NavAggregateAttSelection attSelection;
    NavAggregateTransSelection transSelection;
};

class NavAggregateAlgorithm final {
   public:
    explicit NavAggregateAlgorithm(const NavAggregateConfig& config);

    void setConfig(const NavAggregateConfig& config);

    AggregateOutput update(std::array<InputNavAttData, MAX_AGG_NAV_MSG> attInputs,
                           std::array<InputNavTransData, MAX_AGG_NAV_MSG> transInputs) const;

   private:
    NavAggregateConfig cfg;  //!< [-] validated configuration (selection indices and message counts)
};

}  // namespace f32

#endif
