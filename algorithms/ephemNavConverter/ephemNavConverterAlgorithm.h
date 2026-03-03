#ifndef F32XMERA_EPHEM_NAV_CONVERTER_ALGORITHM_H
#define F32XMERA_EPHEM_NAV_CONVERTER_ALGORITHM_H

#include "ephemNavConverterTypes.h"

#include <Eigen/Core>

/*! @brief The ephemNavConverter algorithm class.*/
class EphemNavConverterAlgorithm {
   public:
    EphemNavConverterAlgorithm() = default;   //!< Constructor
    ~EphemNavConverterAlgorithm() = default;  //!< Destructor

    static OutputNavTransData update(const InputEphemerisData& ephemerisInput);
};

#endif
