#ifndef MESSAGECONVERSIONHERLPERS_H
#define MESSAGECONVERSIONHERLPERS_H

#include "architecture/msgPayloadDef/AttGuidMsgPayload.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"

#include "architecture/msgPayloadDef/AttRefMsgPayload.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"

#include "architecture/msgPayloadDef/NavAttMsgPayload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"

template <size_t N>
void convertArray(const double (&src)[N], float (&dst)[N]) {
    for (size_t i = 0; i < N; ++i) dst[i] = static_cast<float>(src[i]);
}

template <size_t N>
void convertArray(const float (&src)[N], double (&dst)[N]) {
    for (size_t i = 0; i < N; ++i) dst[i] = static_cast<double>(src[i]);
}

template <size_t M, size_t N>
void convert2dArray(const double (&src)[M][N], float (&dst)[M][N]) {
    for (size_t i = 0; i < M; ++i)
        for (size_t j = 0; j < N; ++j) dst[i][j] = static_cast<float>(src[i][j]);
}

template <size_t M, size_t N>
void convert2dArray(const float (&src)[M][N], double (&dst)[M][N]) {
    for (size_t i = 0; i < M; ++i)
        for (size_t j = 0; j < N; ++j) dst[i][j] = static_cast<double>(src[i][j]);
}

inline void convert(const AttGuidMsgPayload& src, AttGuidMsgF32Payload& dst) {
    convertArray(src.sigma_BR, dst.sigma_BR);
    convertArray(src.omega_BR_B, dst.omega_BR_B);
    convertArray(src.omega_RN_B, dst.omega_RN_B);
    convertArray(src.domega_RN_B, dst.domega_RN_B);
}

inline void convert(const AttGuidMsgF32Payload& src, AttGuidMsgPayload& dst) {
    convertArray(src.sigma_BR, dst.sigma_BR);
    convertArray(src.omega_BR_B, dst.omega_BR_B);
    convertArray(src.omega_RN_B, dst.omega_RN_B);
    convertArray(src.domega_RN_B, dst.domega_RN_B);
}

inline void convert(const AttRefMsgPayload& src, AttRefMsgF32Payload& dst) {
    convertArray(src.sigma_RN, dst.sigma_RN);
    convertArray(src.omega_RN_N, dst.omega_RN_N);
    convertArray(src.domega_RN_N, dst.domega_RN_N);
}

inline void convert(const AttRefMsgF32Payload& src, AttRefMsgPayload& dst) {
    convertArray(src.sigma_RN, dst.sigma_RN);
    convertArray(src.omega_RN_N, dst.omega_RN_N);
    convertArray(src.domega_RN_N, dst.domega_RN_N);
}

inline void convert(const NavAttMsgPayload& src, NavAttMsgF32Payload& dst) {
    dst.timeTag = static_cast<float>(src.timeTag);
    convertArray(src.sigma_BN, dst.sigma_BN);
    convertArray(src.omega_BN_B, dst.omega_BN_B);
    convertArray(src.vehSunPntBdy, dst.vehSunPntBdy);
}

inline void convert(const NavAttMsgF32Payload& src, NavAttMsgPayload& dst) {
    dst.timeTag = static_cast<double>(src.timeTag);
    convertArray(src.sigma_BN, dst.sigma_BN);
    convertArray(src.omega_BN_B, dst.omega_BN_B);
    convertArray(src.vehSunPntBdy, dst.vehSunPntBdy);
}

#include "architecture/msgPayloadDef/STSensorMsgPayload.h"
#include "msgPayloadDef/STSensorMsgF32Payload.h"

#include "architecture/msgPayloadDef/STAttMsgPayload.h"
#include "msgPayloadDef/STAttMsgF32Payload.h"

inline void convert(const STSensorMsgPayload& src, STSensorMsgF32Payload& dst) {
    dst.timeTag = static_cast<float>(src.timeTag);
    convertArray(src.qInrtl2Case, dst.qInrtl2Case);
    convertArray(src.omega_CN_C, dst.omega_CN_C);
}

inline void convert(const STSensorMsgF32Payload& src, STSensorMsgPayload& dst) {
    dst.timeTag = static_cast<double>(src.timeTag);
    convertArray(src.qInrtl2Case, dst.qInrtl2Case);
    convertArray(src.omega_CN_C, dst.omega_CN_C);
}

inline void convert(const STAttMsgPayload& src, STAttMsgF32Payload& dst) {
    dst.timeTag = static_cast<float>(src.timeTag);
    convertArray(src.MRP_BdyInrtl, dst.MRP_BdyInrtl);
    convertArray(src.omega_BN_B, dst.omega_BN_B);
    convertArray(src.dcm_CB, dst.dcm_CB);
}

inline void convert(const STAttMsgF32Payload& src, STAttMsgPayload& dst) {
    dst.timeTag = static_cast<double>(src.timeTag);
    convertArray(src.MRP_BdyInrtl, dst.MRP_BdyInrtl);
    convertArray(src.omega_BN_B, dst.omega_BN_B);
    convertArray(src.dcm_CB, dst.dcm_CB);
}

#endif  // MESSAGECONVERSIONHERLPERS_H
