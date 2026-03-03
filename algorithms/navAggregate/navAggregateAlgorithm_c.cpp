#include "navAggregateAlgorithm_c.h"
#include "navAggregateAlgorithm.h"

#include <array>

uint32_t NavAggregateAlgorithm_getMaxAggNavMsg(void) { return MAX_AGG_NAV_MSG; }

NavAggregateAlgorithm* NavAggregateAlgorithm_create(void) {
    return reinterpret_cast<NavAggregateAlgorithm*>(new ::NavAggregateAlgorithm());
}

void NavAggregateAlgorithm_destroy(NavAggregateAlgorithm* self) {
    delete reinterpret_cast<::NavAggregateAlgorithm*>(self);
}

AggregateOutput NavAggregateAlgorithm_update(NavAggregateAlgorithm* self,
                                             const InputNavAttData* attInputs,
                                             const InputNavTransData* transInputs) {
    // Convert C arrays to std::array for C++ algorithm call
    std::array<InputNavAttData, MAX_AGG_NAV_MSG> attArray;
    std::array<InputNavTransData, MAX_AGG_NAV_MSG> transArray;

    for (uint32_t i = 0; i < MAX_AGG_NAV_MSG; ++i) {
        attArray[i] = attInputs[i];
        transArray[i] = transInputs[i];
    }

    return reinterpret_cast<::NavAggregateAlgorithm*>(self)->update(attArray, transArray);
}

void NavAggregateAlgorithm_setAttTimeIdx(NavAggregateAlgorithm* self, uint32_t idx) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setAttTimeIdx(idx);
}

uint32_t NavAggregateAlgorithm_getAttTimeIdx(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getAttTimeIdx();
}

void NavAggregateAlgorithm_setTransTimeIdx(NavAggregateAlgorithm* self, uint32_t idx) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setTransTimeIdx(idx);
}

uint32_t NavAggregateAlgorithm_getTransTimeIdx(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getTransTimeIdx();
}

void NavAggregateAlgorithm_setAttIdx(NavAggregateAlgorithm* self, uint32_t idx) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setAttIdx(idx);
}

uint32_t NavAggregateAlgorithm_getAttIdx(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getAttIdx();
}

void NavAggregateAlgorithm_setRateIdx(NavAggregateAlgorithm* self, uint32_t idx) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setRateIdx(idx);
}

uint32_t NavAggregateAlgorithm_getRateIdx(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getRateIdx();
}

void NavAggregateAlgorithm_setPosIdx(NavAggregateAlgorithm* self, uint32_t idx) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setPosIdx(idx);
}

uint32_t NavAggregateAlgorithm_getPosIdx(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getPosIdx();
}

void NavAggregateAlgorithm_setVelIdx(NavAggregateAlgorithm* self, uint32_t idx) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setVelIdx(idx);
}

uint32_t NavAggregateAlgorithm_getVelIdx(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getVelIdx();
}

void NavAggregateAlgorithm_setDvIdx(NavAggregateAlgorithm* self, uint32_t idx) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setDvIdx(idx);
}

uint32_t NavAggregateAlgorithm_getDvIdx(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getDvIdx();
}

void NavAggregateAlgorithm_setSunIdx(NavAggregateAlgorithm* self, uint32_t idx) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setSunIdx(idx);
}

uint32_t NavAggregateAlgorithm_getSunIdx(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getSunIdx();
}

void NavAggregateAlgorithm_setAttMsgCount(NavAggregateAlgorithm* self, uint32_t msgCount) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setAttMsgCount(msgCount);
}

uint32_t NavAggregateAlgorithm_getAttMsgCount(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getAttMsgCount();
}

void NavAggregateAlgorithm_setTransMsgCount(NavAggregateAlgorithm* self, uint32_t msgCount) {
    reinterpret_cast<::NavAggregateAlgorithm*>(self)->setTransMsgCount(msgCount);
}

uint32_t NavAggregateAlgorithm_getTransMsgCount(const NavAggregateAlgorithm* self) {
    return reinterpret_cast<const ::NavAggregateAlgorithm*>(self)->getTransMsgCount();
}
