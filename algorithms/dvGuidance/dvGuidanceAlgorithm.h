#ifndef F32XMERA_DV_GUIDANCE_ALGORITHM_H
#define F32XMERA_DV_GUIDANCE_ALGORITHM_H

#include "dvGuidanceTypes.h"
#include <stdint.h>
#include <Eigen/Core>

class DvGuidanceAlgorithm final {
   public:
    explicit DvGuidanceAlgorithm(const DvGuidanceConfig& config);

    void setConfig(const DvGuidanceConfig& config);

    DvGuidanceOutput update(const Eigen::Vector3f& dvInrtlCmd,
                            const Eigen::Vector3f& dvRotVecUnit,
                            float dvRotVecMag,
                            uint64_t burnStartTime,
                            uint64_t callTime) const;

   private:
    DvGuidanceConfig cfg;
};

#endif
