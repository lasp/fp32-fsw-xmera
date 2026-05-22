#ifndef F32XMERA_DV_ACCUMULATION_TYPES_H
#define F32XMERA_DV_ACCUMULATION_TYPES_H

#include "utilities/algorithmCShimTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief POD mirror of the C++ DvAccumulationOutput. */
typedef struct {
    double timeTag;          /*!< [s]   time-tag of the most-recently-ingested sample */
    Vector3f_c vehAccumDV_B; /*!< [m/s] accumulated Delta-V in body-frame components */
} DvAccumulationOutput_c;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* F32XMERA_DV_ACCUMULATION_TYPES_H */
