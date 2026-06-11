// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_FLYBY_POINT_TYPES_H
#define F32XMERA_FLYBY_POINT_TYPES_H

#include "utilities/plainCAlgorithmDataTypes.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double data[3];
} Vector3d_c;

typedef struct {
    double timeBetweenFilterData;
    float toleranceForCollinearity;
    int signOfOrbitNormalFrameVector;
    float maxRateThreshold;
    float maxAccelerationThreshold;
    float positionKnowledgeSigma;
} FlybyPointConfig_c;

typedef struct {
    Vector3f_c sigma_RN;
    Vector3f_c omega_RN_N;
    Vector3f_c domega_RN_N;
    bool collinearityTrigger;
    bool maxRateTrigger;
    bool maxAccelerationTrigger;
    bool positionKnowledgeExceedTrigger;
    bool validOutput;
} FlybyPointOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // F32XMERA_FLYBY_POINT_TYPES_H
