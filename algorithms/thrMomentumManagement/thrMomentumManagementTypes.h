#ifndef F32XMERA_THR_MOMENTUM_MANAGEMENT_TYPES_H
#define F32XMERA_THR_MOMENTUM_MANAGEMENT_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of reaction wheels handled at the C boundary. Must match RW_EFF_CNT in
   msgPayloadDef/definitions.h (enforced by a static_assert in the adapter). */
#define THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW 36

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_THR_MOMENTUM_MANAGEMENT_TYPES_H */
