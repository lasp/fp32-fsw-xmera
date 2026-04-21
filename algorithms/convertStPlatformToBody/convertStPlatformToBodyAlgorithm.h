/*
 MIT License

 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_ALGORITHM_H
#define F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_ALGORITHM_H

#include "convertStPlatformToBodyTypes.h"

#include <Eigen/Core>

class ConvertStPlatformToBodyAlgorithm {
   public:
    StAttitudeOutput update(StSensorInput& stSensorIn) const;
    void setDcmCB(const Eigen::Matrix3f& dcm_CB);
    const Eigen::Matrix3f& getDcmCB() const;

   private:
    Eigen::Matrix3f dcm_CB{Eigen::Matrix3f::Identity()};
};

#endif
