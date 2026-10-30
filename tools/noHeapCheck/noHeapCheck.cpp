// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Asserts that the flight algorithms perform no Eigen heap allocation, at
// construction or in steady state.
//
// Two counters, because a hosted build alone does not answer the question:
//
//   heap        - real malloc calls, via -Wl,--wrap=malloc.
//   allocaSites - hits on Eigen's stack-temporary macro, via
//                 -DEIGEN_ALLOCA=probeAlloca. Hosted builds route those to
//                 alloca; the freestanding target routes the same sites to
//                 aligned_malloc, so this counter is the target's heap count.
//
// A negative control runs last. If it does not trip both counters the
// instrumentation is dead and the check fails, so a clean run cannot be a
// false negative.

#include "probeAlloca.h"

#include <flybyFilterAlgorithm.h>
#include <forceTorqueThrForceMappingAlgorithm.h>
#include <inertialFilterAlgorithm.h>
#include <sunlineFilterAlgorithm.h>

#include <Eigen/Dense>

#include <cstdio>
#include <cstdlib>
#include <exception>

extern "C" void* __real_malloc(size_t);
extern "C" void __real_free(void*);

namespace {

long g_heap = 0;
long g_alloca = 0;
bool g_on = false;
int g_failures = 0;

long phaseHeap = 0;
long phaseAlloca = 0;

void begin() {
    phaseHeap = g_heap;
    phaseAlloca = g_alloca;
    g_on = true;
}

//! Ends a measured phase and fails the run if either counter moved.
void endExpectClean(char const* what) {
    g_on = false;
    long const heap = g_heap - phaseHeap;
    long const sites = g_alloca - phaseAlloca;
    bool const ok = (heap == 0) && (sites == 0);
    std::printf("%-40s heap=%-4ld allocaSites=%-4ld %s\n", what, heap, sites, ok ? "ok" : "FAIL");
    if (!ok) {
        ++g_failures;
    }
}

}  // namespace

extern "C" void* __wrap_malloc(size_t n) {
    if (g_on) {
        ++g_heap;
    }
    return __real_malloc(n);
}

extern "C" void __wrap_free(void* p) { __real_free(p); }

// Bump allocator standing in for alloca: alloca cannot be a real function, and
// the buffer is never reclaimed, which is fine for a short-lived check.
extern "C" void* probeAlloca(std::size_t n) {
    if (g_on) {
        ++g_alloca;
    }
    static char buf[1 << 22];
    static std::size_t off = 0;
    char* p = buf + off;
    off += (n + 15U) & ~static_cast<std::size_t>(15);
    return p;
}

namespace inf = filtering::inertialFilter;
namespace snf = filtering::sunlineFilter;
namespace fbf = filtering::flybyFilter;

namespace {

constexpr int updateCycles = 50;

void checkForceTorqueThrForceMapping() {
    ThrusterArrayConfiguration thr{};
    thr.numThrusters = 8;
    float const dirs[8][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {0, 1, 0}};
    float const pos[8][3] = {
        {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {1, 1, -1}, {-1, -1, 1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, -1}};
    for (int i = 0; i < 8; ++i) {
        for (int k = 0; k < 3; ++k) {
            thr.thrusters[i].tHat_B[k] = dirs[i][k];
            thr.thrusters[i].r_TB_B[k] = pos[i][k];
        }
    }
    // The layout spans all six axes (rank 6, condition number 4.1), so every axis can be selected.
    std::array<bool, 6> const axes{true, true, true, true, true, true};
    Eigen::Vector3f const com(0.0F, 0.0F, 0.0F);

    begin();
    auto const cfg = ForceTorqueThrForceMappingConfig::create(thr, com, axes);
    ForceTorqueThrForceMappingAlgorithm const alg(cfg);
    endExpectClean("forceTorqueThrForceMapping construct");

    begin();
    for (int i = 0; i < updateCycles; ++i) {
        auto const out = alg.update(Eigen::Vector3f(0.1F, 0.2F, 0.3F), Eigen::Vector3f(0.0F, 0.0F, 0.1F));
        (void)out.sum();
    }
    endExpectClean("forceTorqueThrForceMapping update");
}

void checkInertialFilter() {
    inf::StateMatrix const q = inf::StateMatrix::Identity() * 1e-6F;
    inf::StateMatrix const p = inf::StateMatrix::Identity() * 1e-2F;
    inf::InertialState const x0{};

    begin();
    auto const cfg = inf::InertialFilterConfig::create(0.1, 2.0, q, x0, p, 1e-3, 1e-4);
    inf::InertialFilterAlgorithm filt(cfg);
    endExpectClean("inertialFilter construct");

    begin();
    double t = 0.0;
    for (int i = 0; i < updateCycles; ++i) {
        t += 0.1;
        inf::StAttData st{};
        st.timeTag = t;
        st.sigma_BN = Eigen::Vector3f(0.01F, 0.02F, 0.03F);
        inf::RateData rd{};
        rd.timeTag = t;
        rd.rate = Eigen::Vector3f(0.001F, 0.002F, 0.003F);
        auto const out = filt.update(t, st, rd);
        (void)out.filterState.state.sum();
    }
    endExpectClean("inertialFilter update");
}

void checkSunlineFilter() {
    snf::StateMatrix const q = snf::StateMatrix::Identity() * 1e-6F;
    snf::StateMatrix const p = snf::StateMatrix::Identity() * 1e-2F;
    // A unit sun heading, and a bias inside [biasLowerBound, biasUpperBound] = [0.5, 1.5].
    snf::SunlineState x0{};
    x0.set<filtering::Position<3>>(Eigen::Vector3f(0.0F, 0.0F, 1.0F));
    x0.set<filtering::Bias<1>>(Eigen::Vector<float, 1>(1.0F));
    Eigen::Matrix<float, snf::MaxCss, 3> nhat = Eigen::Matrix<float, snf::MaxCss, 3>::Zero();
    for (int i = 0; i < snf::MaxCss; ++i) {
        nhat(i, i % 3) = 1.0F;
    }
    Eigen::Vector<float, snf::MaxCss> const scale = Eigen::Vector<float, snf::MaxCss>::Ones();

    begin();
    auto const cfg = snf::SunlineFilterConfig::create(0.1, 2.0, q, x0, p, 0.5, 1.5, nhat, scale, 0.1, 1e-3, 1e-4);
    snf::SunlineFilterAlgorithm filt(cfg);
    endExpectClean("sunlineFilter construct");

    begin();
    double t = 0.0;
    for (int i = 0; i < updateCycles; ++i) {
        t += 0.1;
        snf::CssData css{};
        css.timeTag = t;
        for (int c = 0; c < snf::MaxCss; ++c) {
            css.cosValues(c) = 0.3F + (0.01F * static_cast<float>(c));
        }
        snf::RateData rd{};
        rd.timeTag = t;
        rd.rate = Eigen::Vector3f(0.001F, 0.002F, 0.003F);
        auto const out = filt.update(t, css, rd);
        (void)out.filterState.state.sum();
    }
    endExpectClean("sunlineFilter update");
}

void checkFlybyFilter() {
    fbf::StateMatrix const q = fbf::StateMatrix::Identity() * 1e-6F;
    fbf::StateMatrix const p = fbf::StateMatrix::Identity() * 1e2F;
    fbf::FlybyState x0{};
    x0.set<filtering::Position<3>>(Eigen::Vector3f(1.0e5F, 2.0e5F, 3.0e5F));
    x0.set<filtering::Velocity<3>>(Eigen::Vector3f(1.0F, 0.5F, -0.5F));

    begin();
    auto const cfg = fbf::FlybyFilterConfig::create(0.1, 2.0, 3.986e14, q, x0, p, 1e-3, 1e-2, 2.5e-3);
    fbf::FlybyFilterAlgorithm filt(cfg);
    endExpectClean("flybyFilter construct");

    begin();
    double t = 0.0;
    for (int i = 0; i < updateCycles; ++i) {
        t += 0.1;
        fbf::HeadingData hd{};
        hd.timeTag = t;
        hd.rhat_BN_N = Eigen::Vector3f(0.267F, 0.535F, 0.802F);
        hd.covar_N = Eigen::Matrix3f::Identity() * 1e-6F;  // a positive definite R, so the update runs
        hd.shat_N = Eigen::Vector3f(1.0F, 0.0F, 0.0F);     // off the line of sight, so the bias rotation runs
        auto const out = filt.update(t, hd);
        (void)out.filterState.state.sum();
    }
    endExpectClean("flybyFilter update");
}

//! Dynamic-size work, which must allocate. Guards against a clean run that is
//! really just dead instrumentation.
void checkNegativeControl() {
    begin();
    Eigen::MatrixXd const a = Eigen::MatrixXd::Identity(60, 60);
    Eigen::MatrixXd const b = Eigen::MatrixXd::Identity(60, 60);
    Eigen::MatrixXd const c = a * b;
    Eigen::HouseholderQR<Eigen::MatrixXd> const qr(c);
    Eigen::MatrixXd const r = qr.matrixQR();
    g_on = false;
    (void)r.sum();

    long const heap = g_heap - phaseHeap;
    long const sites = g_alloca - phaseAlloca;
    bool const ok = (heap > 0) && (sites > 0);
    std::printf(
        "%-40s heap=%-4ld allocaSites=%-4ld %s\n", "negative control (must allocate)", heap, sites, ok ? "ok" : "FAIL");
    if (!ok) {
        std::printf("  instrumentation is not counting; the results above prove nothing\n");
        ++g_failures;
    }
}

}  // namespace

int main() {
    try {
        checkForceTorqueThrForceMapping();
        checkInertialFilter();
        checkSunlineFilter();
        checkFlybyFilter();
    } catch (std::exception const& e) {
        g_on = false;
        std::printf("configuration rejected, check did not run: %s\n", e.what());
        return 2;
    }
    checkNegativeControl();

    std::printf("%s\n", g_failures == 0 ? "PASS: no Eigen heap allocation" : "FAIL: Eigen heap allocation detected");
    return g_failures == 0 ? 0 : 1;
}
