#ifndef F32XMERA_EPHEM_NAV_CONVERTER_ALGORITHM_H
#define F32XMERA_EPHEM_NAV_CONVERTER_ALGORITHM_H

#include <Eigen/Core>

/*! Struct containing the ephemeris inputs needed by the algorithm. */
struct InputEphemerisData {
    double timeTag{};
    Eigen::Vector3d r_BdyZero_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_BdyZero_N = Eigen::Vector3d::Zero();
};

/*! Struct containing the translational navigation output produced by the algorithm. */
struct OutputNavTransData {
    double timeTag{};
    Eigen::Vector3d r_BN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_BN_N = Eigen::Vector3d::Zero();
};

/*! @brief The ephemNavConverter algorithm class.*/
class EphemNavConverterAlgorithm {
   public:
    EphemNavConverterAlgorithm() = default;   //!< Constructor
    ~EphemNavConverterAlgorithm() = default;  //!< Destructor

    static OutputNavTransData update(const InputEphemerisData& ephemerisInput);
};

#endif
