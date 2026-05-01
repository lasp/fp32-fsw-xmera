// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ARRAY_EFFECTOR_LOCK_F32_H
#define ARRAY_EFFECTOR_LOCK_F32_H

#include "definitions.h"

/*! @brief Structure used to define per-effector lock flags */
typedef struct {
    int effectorLockFlag[MAX_EFF_CNT];  //!< effector lock flag; 0 = unlocked, 1 = locked
} ArrayEffectorLockMsgF32Payload;

#endif
