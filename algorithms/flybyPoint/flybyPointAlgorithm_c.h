// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_FLYBY_POINT_ALGORITHM_C_H
#define F32XMERA_FLYBY_POINT_ALGORITHM_C_H

#include "flybyPointTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FlybyPointAlgorithmHandle FlybyPointAlgorithmHandle;

FlybyPointAlgorithmHandle* FlybyPointAlgorithm_create(const FlybyPointConfig_c* config);
void FlybyPointAlgorithm_destroy(FlybyPointAlgorithmHandle* self);
void FlybyPointAlgorithm_reset(FlybyPointAlgorithmHandle* self);
void FlybyPointAlgorithm_setConfig(FlybyPointAlgorithmHandle* self, const FlybyPointConfig_c* config);
FlybyPointOutput_c FlybyPointAlgorithm_updateState(FlybyPointAlgorithmHandle* self,
                                                   uint64_t currentSimNanos,
                                                   Vector3d_c r_BN_N,
                                                   Vector3d_c v_BN_N);

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // F32XMERA_FLYBY_POINT_ALGORITHM_C_H
