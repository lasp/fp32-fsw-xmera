#ifndef F32XMERA_DV_GUIDANCE_ALGORITHM_H
#define F32XMERA_DV_GUIDANCE_ALGORITHM_H

#include "dvGuidanceTypes.h"
#include <stdint.h>
#include <Eigen/Core>

class DvGuidanceAlgorithm final {
   public:
    DvGuidanceAlgorithm() = default;

    DvGuidanceOutput update(const Eigen::Vector3d& dvInrtlCmd,
                            const Eigen::Vector3d& dvRotVecUnit,
                            double dvRotVecMag,
                            uint64_t burnStartTime,
                            uint64_t callTime) const;
};

#endif
