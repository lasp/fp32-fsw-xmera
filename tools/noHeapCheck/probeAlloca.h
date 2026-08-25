// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Force-included into every TU of the check so -DEIGEN_ALLOCA=probeAlloca has a
// declaration in scope wherever Eigen expands its stack-temporary macro.

#ifndef FP32_XMERA_TOOLS_NOHEAPCHECK_PROBEALLOCA_H
#define FP32_XMERA_TOOLS_NOHEAPCHECK_PROBEALLOCA_H

#include <cstddef>

extern "C" void* probeAlloca(std::size_t);

#endif
