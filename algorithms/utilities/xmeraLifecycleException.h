// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef F32XMERA_XMERA_LIFECYCLE_EXCEPTION_H
#define F32XMERA_XMERA_LIFECYCLE_EXCEPTION_H

#include <stdexcept>

// Thrown by Xmera adapters when they detect a lifecycle violation -- typically updateState() being
// called before reset(), so the algorithm has never been constructed.
//
// This header is for adapter use (hosted environment) only. Algorithm production code targeting
// freestanding C++ must not throw, so should not include this header.
class XmeraLifecycleException : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

#endif  // F32XMERA_XMERA_LIFECYCLE_EXCEPTION_H
