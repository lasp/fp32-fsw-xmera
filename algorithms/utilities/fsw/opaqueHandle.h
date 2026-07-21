#ifndef F32XMERA_UTILITIES_FSW_OPAQUEHANDLE_H
#define F32XMERA_UTILITIES_FSW_OPAQUEHANDLE_H

#include <utility>

// Bridges an opaque C handle (a forward-declared, never-completed struct) to its C++
// implementation for the Ada/FFI shims. reinterpret_cast is required because the handle
// type stays incomplete, and _create/_destroy own the allocation across a C ABI boundary
// where unique_ptr/gsl::owner cannot be carried. Those unavoidable-but-flagged casts and
// the owning new/delete are confined and audited here so the shims stay suppression-free.

namespace fsw {

//! Allocate an Impl and return it as an opaque Handle* (for <Name>Algorithm_create).
template <class Impl, class Handle, class... Args>
Handle* createHandle(Args&&... args) {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory, cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<Handle*>(new Impl(std::forward<Args>(args)...));
}

//! Recover the Impl* (or const Impl*) behind a Handle* (for every accessor/mutator).
template <class Impl, class Handle>
Impl* fromHandle(Handle* handle) {
    return reinterpret_cast<Impl*>(handle);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
}

//! Destroy the Impl behind a Handle* (for <Name>Algorithm_destroy).
template <class Impl, class Handle>
void deleteHandle(Handle* handle) {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory, cppcoreguidelines-pro-type-reinterpret-cast)
    delete reinterpret_cast<Impl*>(handle);
}

}  // namespace fsw

#endif
