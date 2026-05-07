# Float32 Numerical Precision Guidelines

**Adamant-Xmera Flight Software Algorithms (fp32-fsw-xmera)**

This document is **normative** -- all new C++ GNC algorithm code in this repository MUST comply with
these guidelines. Existing code should be brought into compliance as it is modified.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [IEEE 754 Float32 Quick Reference](#2-ieee-754-float32-quick-reference)
3. [Precision Policy: When to Use Which Type](#3-precision-policy-when-to-use-which-type)
4. [safeMath: Mandatory Safe Function Wrappers](#4-safemath-mandatory-safe-function-wrappers)
5. [Numerical Patterns and Their Risks](#5-numerical-patterns-and-their-risks)
6. [Eigen with Float32](#6-eigen-with-float32)
7. [Tolerance Derivation Methodology](#7-tolerance-derivation-methodology)
8. [Past Examples: Precision Failures in This Codebase](#8-past-examples-stories-precision-failures-in-this-codebase)
9. [Fuzz Testing for Precision Validation](#9-fuzz-testing-for-precision-validation)
10. [Compile-Time and Runtime Checks](#10-compile-time-and-runtime-checks)
11. [Static Analysis Compliance](#11-static-analysis-compliance)
12. [Pre-Implementation Checklist](#12-pre-implementation-checklist)
13. [Appendix A: Compiler Flags](#appendix-a-compiler-flags)

---

## 1. Introduction

### 1.1 Purpose and Scope

This document establishes
- precision rules
- numerical pattern guidance
- safeMath policy
- fuzz testing methodology
- Eigen usage guidelines
- static analysis compliance,

for all C++23 GNC algorithm code in this repository.

**In scope:** All code under `algorithms/` and `architecture/utilities/`, their unit tests, and fuzz
tests.

**Out of scope:** Performance optimization, Python test code, Xmera simulation tool.

### 1.2 Audience

- **GNC engineers** who design algorithms and need to understand which numerical patterns are fragile
  in single precision (32 bit floating point), how to identify precision-sensitive operations,
  and how to derive appropriate test tolerances.


### 1.3 Why Float32?

The target platform is a RISC-V 32-bit processor with the **F extension** (hardware single-precision
floating-point unit). Double-precision arithmetic is available via **software emulation** but carries
substantial in execution time. Float32 is the primary computation type because of this hardware constraint, not as a performance
preference. The guidelines in this document exist to ensure numerical correctness within float32's
inherent limitations.

---

## 2. IEEE 754 Float32 Quick Reference

### 2.1 What You Get

| Property                | float (32-bit)        | double (64-bit)        |
|-------------------------|-----------------------|------------------------|
| Mantissa bits           | 23                    | 52                     |
| Decimal digits          | ~7.2                  | ~15.9                  |
| Machine epsilon         | 1.19e-7               | 2.22e-16               |
| Range                   | +/- 3.4e38            | +/- 1.8e308            |
| Smallest normal         | 1.18e-38              | 2.22e-308              |

### 2.2 Practical GNC Consequences

1. **Position resolution.** LEO orbital position (~7e6 m) has ~0.7 m resolution in float vs
   ~1.6e-9 m in double. This is why `NavTransMsgF32Payload.r_BN_N` and
   `EphemerisMsgF32Payload.r_BdyZero_N` use `double`.

2. **Angular precision.** Angles near 0 or pi lose precision in trigonometric operations. For
   example, `cos(x)` near `x = 0` computes `1 - x^2/2`, and the subtraction from 1 loses
   significant digits when `x` is small.

3. **Error accumulation.** Each floating-point operation introduces up to 0.5 ULP of rounding error.
   Over a chain of `n` operations, error accumulates roughly as `O(n * epsilon)`. For integration
   loops, recurrence relations, and Kalman filter updates, this compounding is the primary precision
   concern.

### 2.3 Key Terms

- **ULP (Unit in the Last Place):** The spacing between two adjacent floating-point numbers at a
  given magnitude. For float, 1 ULP at magnitude 1.0 is ~1.19e-7.
- **Machine epsilon:** The smallest value `e` such that `1.0 + e != 1.0` in floating-point. For
  float: 1.19e-7. For double: 2.22e-16.
- **Condition number:** The ratio of relative output change to relative input change. A condition
  number of 10^k means you lose ~k digits of accuracy. An operation with condition number 10^4
  applied to float (~7 digits) yields only ~3 reliable digits.
- **Catastrophic cancellation:** Loss of significant digits when subtracting two nearly equal
  numbers. For example, if `a = 1.0000001` and `b = 1.0000000` in float, the result
  `a - b = 1e-7` has only ~1 significant digit.

---

## 3. Precision Policy: When to Use Which Type

### 3.1 The Default: float (float32)

All algorithm computation, message payload fields for attitude/rates/forces, control gains, and
sensor measurements use `float`. This is the default and requires no justification. All message
payload structures use the `F32` suffix convention (e.g., `NavAttMsgF32Payload`,
`AttGuidMsgF32Payload`).

### 3.2 Approved Uses of double

The following are the **only** approved uses of `double` in flight code. Each represents a case
where float's ~7 decimal digits are insufficient:

1. **Time tags and time arithmetic.** Epoch times in seconds since J2000 can exceed 1e9, and
   sub-millisecond resolution requires >12 significant digits. Examples: `NavAttMsgF32Payload.timeTag`,
   `TDBVehicleClockCorrelationMsgF32Payload.ephemerisTime`, the `kNano2Sec` conversion chain in
   `timeConstants.h`.

2. **Position and velocity in meters.** LEO position magnitudes (~7e6 m) need better than 1 m
   resolution for orbit propagation. Examples: `NavTransMsgF32Payload.r_BN_N`,
   `EphemerisMsgF32Payload.r_BdyZero_N`.

3. **Orbital element computation.** Kepler's equation Newton-Raphson iteration uses
   `kTolerance = 1e-9`, which requires >9 significant digits to converge. The entire
   `orbitalMotion.hpp` operates in double for this reason.

4. **Chebyshev coefficient storage.** Ephemeris fit coefficients in `ChebyshevFitArc` are
   ground-computed to high precision and stored as `double` to preserve that accuracy.

### 3.3 Adding a New double Use

To add a new use of `double` in flight code:

1. Document the precision requirement: show that float's ~7 digits is insufficient for the
   specific value range and required accuracy.
2. Obtain code review approval with the precision analysis.
3. Add a comment at the declaration site explaining why `double` is necessary.

### 3.4 Mixed-Precision Boundaries

**Hard rule:** Every narrowing conversion from `double` to `float` MUST use explicit
`static_cast<float>()` with a comment explaining why the narrowing is acceptable.

```cpp
// Good: explicit cast with rationale
auto dt = static_cast<float>(static_cast<double>(callTime - priorTime) * kNano2Sec);
// Time delta is small enough (<~1000 s) that float precision suffices

// Bad: implicit narrowing
float dt = static_cast<double>(callTime - priorTime) * kNano2Sec;  // implicit double->float
```

### 3.5 Literal Suffix Convention

All float-precision literals MUST use the `F` suffix. Omitting it creates a `double` literal, which
causes implicit promotion when used in float expressions.

```cpp
// Correct (validDcmCheck.h)
inline constexpr float kToleranceF = 1.0e-5F;

// Incorrect (validInertiaCheck.h:6) -- missing F suffix
constexpr float tolerance = 1e-6;   // 1e-6 is double, implicitly narrowed to float
```

---

## 4. safeMath: Mandatory Safe Function Wrappers

### 4.1 The Rule

**All trigonometric, inverse trigonometric, hyperbolic, and sqrt calls in flight code MUST use the
corresponding safeMath wrapper.** Direct calls to `sinf()`, `cosf()`, `acosf()`, `sqrtf()`,
`atan2f()`, etc. are prohibited. No exceptions.

This rule ensures that **no NaN or Inf can propagate through mathematical functions**, regardless of
input. Every safeMath wrapper returns a finite output for any finite input.

### 4.2 What safeMath Provides

All wrappers are defined in `algorithms/utilities/safeMath.h`:

| Wrapper (float)  | Wrapper (double) | Domain Clamping                          | Output Bounds              |
|-------------------|------------------|------------------------------------------|----------------------------|
| `safeSinf`        | `safeSin`        | None                                     | [-1, 1]                    |
| `safeCosf`        | `safeCos`        | None                                     | [-1, 1]                    |
| `safeTanf`        | `safeTan`        | Input clamped to [-pi/2+eps, pi/2-eps]   | [-cot(eps), cot(eps)]      |
| `safeAsinf`       | `safeAsin`       | Input clamped to [-1, 1]                 | [-pi/2, pi/2]              |
| `safeAcosf`       | `safeAcos`       | Input clamped to [-1, 1]                 | [0, pi]                    |
| `safeAtanf`       | `safeAtan`       | None                                     | [-pi/2, pi/2]              |
| `safeAtan2f`      | `safeAtan2`      | Returns 0 when both args are zero        | (-pi, pi]                  |
| `safeSqrtf`       | `safeSqrt`       | Negative inputs clamped to 0             | [0, inf)                   |
| `safeSinHf`       | `safeSinH`       | None                                     | [-1/eps, 1/eps]            |
| `safeCosHf`       | `safeCosH`       | None                                     | [1, 1/eps]                 |
| `safeTanHf`       | `safeTanH`       | None                                     | [-1, 1]                    |
| `safeAtanHf`      | `safeAtanH`      | Input clamped to [-(1-eps), (1-eps)]     | [atanh(-(1-eps)), atanh(1-eps)] |

Where `eps` = `std::numeric_limits<T>::epsilon()`.

### 4.3 When to Add New Wrappers

If a new math function is needed (e.g., `logf`, `expf`), follow the existing pattern:

1. Guard against NaN/Inf inputs -- return a safe default.
2. Clamp inputs to the valid domain.
3. Bounds-check the output defensively.
4. Provide both `float` and `double` variants.

---

## 5. Numerical Patterns and Their Risks

This section categorizes the numerical patterns that appear across this codebase and are sensitive to
float32 precision. Each subsection describes the pattern, explains why it is dangerous, lists the
algorithms that exhibit it, and prescribes the mitigation.

### 5.1 Division by Small Values

**Pattern:** Expressions `a / b` where `b` can approach zero. In float, values near epsilon
(1.19e-7) cause catastrophic loss of significance or overflow.

**Where it appears:**

| Function | File                          | Divisor | Singularity Condition |
|----------|-------------------------------|---------|-----------------------|
| `mrpShadow` | `rigidBodyKinematics.hpp:38`  | `mrp.squaredNorm()` | MRP at origin (zero rotation) |
| `binvMrp` | `rigidBodyKinematics.hpp:78`  | `(1 + dotProd)^2` | MRP approaching anti-parallel composition |
| `binvPrv` | `rigidBodyKinematics.hpp:104` | `norm^2`, `norm^3` | PRV at identity rotation |
| `binvEulerAngles321` | `rigidBodyKinematics.hpp:135` | `cos(euler2)` | Gimbal lock at pitch = +/-pi/2 |
| `dcmToEp` | `rigidBodyKinematics.hpp:273` | `4.0 * ep[i]` | When selected EP component is near zero |
| `dcmToMrp` | `rigidBodyKinematics.hpp:325` | `1.0 + ep(0)` | When principal EP is near -1 |
| `epToPrv` | `rigidBodyKinematics.hpp:336` | `sin(angle)` | PRV angle at 0 or 2*pi |
| `mrpToDcm` | `rigidBodyKinematics.hpp:550` | `(1 + mrp.dot(mrp))^2` | Not a practical concern (always >= 1) |
| `cartesianStateToElements` | `orbitalMotion.hpp`           | `h = norm(r x v)` | Rectilinear orbits (h = 0) |
| `elementsToCartesianState` | `orbitalMotion.hpp`           | `sqrt(mu * p)` | Parabolic/degenerate orbits (p -> 0) |

**Mitigation:** Add epsilon guards before division. Where feasible, restructure to avoid division
entirely (e.g., use `atan2(y, x)` instead of `atan(y/x)`). Currently, only `epToPrv` (guard at
1e-12) and `mrpToPrv` (guard at 1e-10) have explicit epsilon guards. Other functions rely on the
caller ensuring valid inputs.

### 5.2 Trigonometric Function Singularities

**Pattern:** Functions like `acos(x)` where `x` can exceed [-1, 1] due to floating-point roundoff,
or `tan(x)` near pi/2.

**Where it appears:**

| Function | File | Operation | Risk |
|----------|------|-----------|------|
| `dcmToEp` | `rigidBodyKinematics.hpp:273` | `sqrt()` of trace-derived values | Argument can go slightly negative |
| `dcmToEulerAngles321` | `rigidBodyKinematics.hpp:364` | `asin(-dcm(0,2))` | DCM entries can exceed [-1,1] in float |
| `epToPrv` | `rigidBodyKinematics.hpp:336` | `acos(ep(0))` | EP normalization error can push ep(0) outside [-1,1] |
| `epToEulerAngles321` | `rigidBodyKinematics.hpp:520` | `asin`, `atan2` | Same gimbal lock concerns |
| `binvPrv` | `rigidBodyKinematics.hpp:104` | `cos(norm)`, `sin(norm)` | Cancellation near norm = 0 |
| `bmatPrv` | `rigidBodyKinematics.hpp:228` | `tan(normPrv/2)` | Singularity at normPrv = pi |

**Mitigation:** Always use safeMath wrappers. `safeAcosf` clamps input to [-1, 1]. `safeSqrtf`
clamps negative inputs to zero. `safeTanf` clamps input away from +/-pi/2.

### 5.3 Iterative Convergence

**Pattern:** Newton-Raphson or similar iterative solvers where the convergence tolerance must be
achievable in the working precision.

**Where it appears:**

| Function | File                    | Tolerance | Max Iterations | Precision |
|----------|-------------------------|-----------|----------------|-----------|
| `meanToEccentricAnomaly` | `orbitalMotion.hpp:114` | `kTolerance = 1e-9` | 200 | double |
| `meanToHyperbolicAnomaly` | `orbitalMotion.hpp:145` | `kTolerance = 1e-9` | 200 | double |

Both solvers use step clamping to prevent divergence near singular regimes:
- Elliptic: step clamped to [-0.5, 0.5] for near-parabolic stability
- Hyperbolic: initial guess clamped to +/-7 to prevent divergence

**Mitigation:** If the convergence tolerance is tighter than ~1e-5, the iteration MUST use `double`.
`float` cannot reliably distinguish values that differ by less than ~1e-7 (relative), so a 1e-9
convergence criterion is impossible in `float`. If `float` output is required, perform the iteration in
double and narrow at the end with an explicit `static_cast<float>()`.

### 5.4 Recurrence Relations and Accumulated Error

**Pattern:** Iterative formulas where each step's error compounds. The canonical example is the
Chebyshev three-term recurrence in `chebyshevUtilities.h:40`:

```cpp
chebyNow = (valueMult * chebyNow) - chebyPrev;
```

Each step performs 2 multiplications and 1 subtraction, introducing ~2 ULP of error. After `n`
steps, the basis polynomial `T_n(x)` carries ~2n ULP of accumulated error.

**Where it appears:**

| Function | File | Steps | Float32 Error at n=20 |
|----------|------|-------|-----------------------|
| `calculateChebyValue` | `chebyshevUtilities.h:28` | n (number of coefficients) | ~40 * 1.19e-7 ~ 5e-6 relative |

**Mitigation:** Bound the maximum degree. Use the L1 norm analysis (Section 7.3) to verify that the
accumulated error is acceptable for the given coefficient magnitudes. If the ratio of the largest to
smallest coefficient exceeds ~1e5, the small coefficients are effectively noise in float32 and the
evaluation should use double.

### 5.5 Norm-Dependent Branching (Threshold Sensitivity)

**Pattern:** Code paths that branch on a computed norm or determinant, where float32 vs double may
compute slightly different values, causing different branches to execute. This produces
discontinuously different results -- not a small error, but a fundamentally different output.

**Where it appears:**

| Function | File | Condition | Threshold |
|----------|------|-----------|-----------|
| `mrp_product` | `eigenMRP.h:454` | `det < 0.01` | Shadow switch on composition determinant |
| `mrp_product` | `eigenMRP.h:460` | `answer.squaredNorm() > 1` | Shadow switch on result norm |
| `mrpSwitch` | `rigidBodyKinematics.hpp:49` | `mrp.squaredNorm() > s * s` | Shadow switch on MRP norm |
| `epToMrp` | `rigidBodyKinematics.hpp:506` | `ep(0) >= 0.0` | Branch on EP sign |

**Mitigation:** Ensure branching thresholds have adequate **hysteresis** relative to float32
precision. When comparing float behavior against a double reference, compare the resulting DCM or
rotation angle -- not the MRP vector directly, since both branches produce valid (but different)
representations of the same rotation. See Past Example 8.1 for a detailed example.

### 5.6 Catastrophic Cancellation

**Pattern:** Subtraction of nearly equal quantities, losing most significant digits.

**Where it appears:**

| Expression | File | Context | Risk |
|-----------|------|---------|------|
| `1 - mrp.dot(mrp)` | `rigidBodyKinematics.hpp` (bmatMrp, mrpToDcm) | Near identity rotation (dot ~ 0), safe. Near unit MRP (dot ~ 1), loses digits. |
| `1 + s1N2*s2N2 - 2*a.dot(b)` | `eigenMRP.h:453` | MRP product determinant -- cancels to near zero when composing nearly inverse rotations |
| `(1 - s1N2) * s2` | `eigenMRP.h:459` | MRP product numerator -- cancels when s1 norm is near 1 |

**Mitigation:** Where possible, use mathematically equivalent reformulations that avoid the
cancellation (e.g., `2*sin^2(x/2)` instead of `1 - cos(x)`). Where reformulation is not feasible,
document the expected magnitude range of the intermediate values and verify with fuzz testing that
the cancellation does not produce unacceptable output error.

### 5.7 Large Dynamic Range Quantities

**Pattern:** Values that span many orders of magnitude, exceeding float32's ~7-digit capacity.

**Where it appears:**

| Quantity | Typical Range | Float32 Resolution |
|----------|---------------|--------------------|
| Orbital position (m) | 1e3 to 1e9 | 0.06 m at LEO, 60 m at GEO |
| Gravitational parameter mu (m^3/s^2) | 3.986e14 (Earth) | ~5e7 m^3/s^2 |
| Epoch time (s since J2000) | 0 to ~8e8 | ~50 s |

**Mitigation:** Keep these computations in double. This is already the case for `orbitalMotion.hpp`
and time tags. Any new algorithm operating on quantities with >7 orders of magnitude dynamic range
must use double (see Section 3.2).

---

## 6. Eigen with Float32

### 6.1 Freestanding Mode

This project uses Eigen with `EIGEN_FREESTANDING=1`, which disables heap allocation, standard
library I/O, and exceptions from Eigen internals. The numerical core (matrix arithmetic,
decompositions, solvers) is unaffected. Fixed-size matrices and vectors work normally.

### 6.2 Safe Operations

The following Eigen operations are safe in float32 for the matrix sizes in this codebase (3x3, 4x3,
3xN for small N):

- **Matrix-vector multiplication:** Error bounded by `O(n * epsilon)` for dimension n. For 3x3,
  this is ~3 * 1.19e-7 ~ 3.6e-7.
- **Dot product:** Error bounded by `O(n * epsilon)` for n-dimensional vectors.
- **Cross product:** Implemented via determinant formula; error ~6 * epsilon.
- **Basic arithmetic (+, -, scalar *):** 1 ULP per operation.
- **Transpose, block operations:** Exact (no arithmetic).

### 6.3 Operations Requiring Care

**`isApprox(other, tolerance)`** -- The default tolerance is
`NumTraits<float>::dummy_precision()` = `1e-5F`. This is a **relative** tolerance, not absolute. It
is used in `validDcmCheck.h:11` to check DCM orthogonality. Be aware that `isApprox` can behave
unexpectedly when comparing against zero -- use absolute tolerance checks for near-zero values.

**`.norm()` and `.squaredNorm()`** -- For vectors with components of very different magnitude, the
sum-of-squares computation can lose precision. `squaredNorm()` avoids the sqrt but has the same
summation issue. This is the root cause of the MRP shadow switch divergence (Section 8.1).

**`.determinant()` for 3x3** -- Computed via the cofactor expansion, which involves subtraction of
products. Susceptible to cancellation for ill-conditioned matrices. The `validDcmCheck.h` tolerance
of `1e-5F` accounts for this.

**`SelfAdjointEigenSolver`** -- Used in `validInertiaCheck.h:15`. Eigenvalue accuracy depends on
the matrix condition number. For well-conditioned inertia tensors (condition number < 100), float32
is adequate.

**Implicit type mixing** -- Eigen blocks this at compile time with `EIGEN_STATIC_ASSERT` (see
`eigenMRP.h:472`). You cannot multiply a `Matrix3f` by a `Matrix3d`. This is a safety feature --
do not circumvent it.

### 6.4 The eigenMRP Extension

`architecture/utilities/eigenMRP.h` provides `MRP<Scalar>`, `MRPf`, `MRPd`. Precision-sensitive
operations:

- **`toRotationMatrix()`** involves `squaredNorm()` and divisions by the square of
  `(1 + squaredNorm)`. Not a practical singularity (denominator >= 1) but accumulates error.
- **`shadow()`** divides by `squaredNorm()`. Undefined for zero MRP -- callers must ensure non-zero.
- **`mrp_product`** (`eigenMRP.h:444`) has two branching points (`det < 0.01` and
  `squaredNorm() > 1`) that are sensitive to precision. See Section 5.5.

---

## 7. Tolerance Derivation Methodology

### 7.1 Philosophy

Tolerances in tests and convergence checks should be **derived** from the algorithm's mathematical
properties and the working precision. Do not copy a tolerance from another algorithm, and do not pick
a round number by trial and error.

### 7.2 Forward Error Analysis Basics

For a chain of `n` floating-point operations, the accumulated relative error is bounded
(approximately) by:

```
relative_error <= n * epsilon * (1 + O(n * epsilon))
```

For float32 with `epsilon = 1.19e-7`:

| Operations | Approx. Relative Error | Typical Tolerance |
|------------|------------------------|-------------------|
| 3          | 3.6e-7                 | 1e-6              |
| 10         | 1.2e-6                 | 1e-5              |
| 30         | 3.6e-6                 | 1e-5              |
| 100        | 1.2e-5                 | 1e-4              |

A safety margin of 3-10x above the theoretical bound is appropriate. This is why `1e-5F` and
`1e-6F` are the most common tolerances in this codebase.

### 7.3 Worked Example: Chebyshev L1 Norm Bound

This example walks through the tolerance derivation for the Chebyshev polynomial fuzz tests in
`test_chebyshevUtilities_fuzz.cpp`.

**The computation.** `calculateChebyValue` (`chebyshevUtilities.h:28`) evaluates a Chebyshev
polynomial of degree `n-1` using the three-term recurrence:

```
T_0(x) = 1,  T_1(x) = x,  T_k(x) = 2x * T_{k-1}(x) - T_{k-2}(x)
```

The final result is `f(x) = sum(c_i * T_i(x))` for `i = 0..n-1`.

**Step 1: Count operations.**
- Recurrence: 1 multiply + 1 multiply-subtract per step = ~3 FP ops/step, for `n-2` steps.
- Coefficient accumulation: 1 multiply-add per coefficient = `n` ops.
- Total: ~4n floating-point operations.

**Step 2: Bound the basis polynomials.**
Every Chebyshev polynomial satisfies `|T_k(x)| <= 1` for `x` in `[-1, 1]`. Therefore the output is
bounded by the L1 norm of the coefficient vector: `|f(x)| <= sum(|c_i|) = L1_norm`.

**Step 3: Compute the error bound.**
The floating-point error in the output is approximately:

```
error <= L1_norm * 4n * epsilon * (1 + O(n * epsilon))
```

For `n = 20` and float32:
```
error <= L1_norm * 80 * 1.19e-7 * (1 + ~2e-5)
       ~ L1_norm * 9.5e-6
```

**Step 4: Set the tolerance.**
The fuzz test in `test_chebyshevUtilities_fuzz.cpp:134` uses:

```cpp
const float bound = l1Norm * (1.0f + 1e-4f) + 1e-6f;
```

- The `1e-4f` relative factor provides ~10x margin over the theoretical `9.5e-6`.
- The `1e-6f` absolute floor handles the case where L1 norm is very small.

**Comparison with double.** The double-precision fuzz test (`line 117`) uses:

```cpp
const double bound = l1Norm * (1.0 + 1e-10) + 1e-15;
```

The `1e-10` relative factor reflects double's epsilon (~2.22e-16) with the same `4n` operation
count: `80 * 2.22e-16 ~ 1.8e-14`, with ~5000x margin.

### 7.4 Worked Example: DCM Orthogonality Tolerance

`validDcmCheck.h` checks `(dcm^T * dcm).isApprox(Identity, 1e-5F)`. Why `1e-5F`?

A 3x3 DCM constructed from MRP via `mrpToDcm` involves approximately 30 floating-point operations
(additions, multiplications, divisions). The expected error in `dcm^T * dcm - I` is:

```
~30 * 1.19e-7 = 3.6e-6
```

The tolerance of `1e-5F` provides approximately 3x margin above this theoretical error.

### 7.5 Guidelines for New Tolerances

1. **Count the operations** in the computation chain from input to the value being compared.
2. **Multiply by epsilon** for the working precision (1.19e-7 for float, 2.22e-16 for double).
3. **Account for condition number** if the computation involves ill-conditioned steps (inversion,
   near-singular matrices). Multiply the operation-count error by the condition number.
4. **Add a safety margin** of 3-10x to account for analysis approximations.
5. **Include an absolute floor** for cases where the expected value is near zero.

### 7.6 Reference Truth: Double as Ground Truth vs. Better Approximation

When comparing float algorithm output against a double-precision reference in tests, recognize the
distinction:

**Double as ground truth** -- For closed-form expressions (DCM construction from MRP, coordinate
transforms, trig identities), double gives the mathematically exact answer to 15+ digits. The
tolerance should reflect only the float implementation's error: `~n_ops * float_epsilon * safety`.

**Double as better approximation** -- For iterative algorithms (Newton-Raphson for Kepler's
equation, UKF convergence), the double solution is merely a closer approximation, not the true
answer. The tolerance must account for both the float error AND the double approximation's residual
error. In practice, for converged iterations, the double residual is negligible compared to float
error, but this should be verified rather than assumed.

---

## 8. Past Examples: Precision Failures in This Codebase

### 8.1 MRP Shadow Set Switch Divergence

**The bug.** The MRP composition in `eigenMRP.h` (`mrp_product`, line 444) computes the result and
then checks `answer.squaredNorm() > 1` to decide whether to map to the shadow set. When the true
squared norm is very close to 1.0, float32 and double can compute values on opposite sides of the
threshold:

```
True squared norm:    0.99999997
Float32 computes:     1.0000001   --> triggers shadow switch
Double computes:      0.999999970 --> no switch
```

Because the shadow set maps `sigma` to `-sigma / |sigma|^2`, the two precisions now produce
fundamentally different MRP vectors. Both represent the **same rotation**, but downstream code that
tracks MRP continuity (e.g., integration) sees a massive discontinuity.

**Root cause.** Float32 computes `squaredNorm()` (a sum of 3 squares) with ~7 digits of precision.
At magnitude ~1.0, the uncertainty is ~1.19e-7. The branching threshold `> 1` has zero hysteresis --
any noise in the 7th digit can flip the branch.

**Lesson.** For any norm-based branching:

1. Ensure the threshold has adequate hysteresis relative to float32 precision.
2. When validating float code against a double reference, **compare the resulting DCM or rotation
   angle**, not the MRP vector. The MRP representation is non-unique due to the shadow set, and
   precision-induced switching is expected behavior.
3. If the algorithm's correctness depends on which shadow set is chosen, the threshold must be
   adjusted for the working precision.

### 8.2 Chebyshev Recurrence Error Accumulation

**The bug.** The `calculateChebyValue` template evaluates Chebyshev polynomials via the three-term
recurrence. For high-degree polynomials (n > ~15) with float32, the accumulated rounding error in
the recurrence can exceed the magnitude of small coefficients, effectively corrupting their
contribution to the sum.

**Root cause.** Each recurrence step `T_k = 2x * T_{k-1} - T_{k-2}` introduces ~2 ULP of error.
After `n` steps, the basis polynomial `T_n(x)` carries ~2n ULP of accumulated error. For n = 20:

```
Error in T_20(x) ~ 40 * 1.19e-7 ~ 5e-6 (relative)
```

When this error is multiplied by coefficient `c_20`, the contribution `c_20 * T_20(x)` has ~5e-6
relative error. If `c_20` is small relative to the total sum (e.g., `c_20 / sum(|c_i|) < 1e-5`),
the error in this term exceeds its contribution -- it becomes numerical noise.

**Lesson.** Use the L1 norm bound (Section 7.3) to verify that a given coefficient set and
polynomial degree are safe at the target precision. If the coefficient dynamic range
(max(|c_i|) / min(|c_i|)) exceeds ~1e5, the smallest coefficients are unreliable in float32 and the
evaluation should use double.

---

## 9. Fuzz Testing for Precision Validation

### 9.1 Philosophy

When a fuzz test discovers a precision failure, the correct response is to **fix the algorithm** --
add a guard, restructure the computation, or use safeMath. Do NOT widen the tolerance to make the
test pass. Widening the tolerance masks real precision problems that could manifest in operation.

### 9.2 Two Fuzz Strategies

#### 9.2.1 Double-Reference Comparison

Compute the same function independently in double and float, then compare results within a
float-precision tolerance.

**When to use:**
- The function has a closed-form expression that can be independently implemented in double.
- You want to verify that the float implementation matches the mathematical intent to within float
  precision.

**Example:** `test_oeStateEphem_helpers.h` implements an independent double-precision reference for
`elementsToCartesianState`, then compares the algorithm's float output against it.

**Caveats:**
- Distinguish "double as ground truth" (closed-form) from "double as better approximation"
  (iterative). See Section 7.6.
- For functions with norm-dependent branching (Section 5.5), the float and double implementations
  may take different branches. Compare the rotation (DCM) not the representation (MRP).

#### 9.2.2 Property-Based Invariant Checking

Verify mathematical properties that must hold regardless of precision: round-trip identities,
conservation laws, symmetries, boundedness.

**When to use:**
- The invariant is more important than the exact value (e.g., "the DCM must be orthogonal" is more
  meaningful than "DCM(0,0) must equal 0.123456...").

**Examples from this codebase:**
- `test_safeMath_fuzz.cpp`: Pythagorean identity (`safeCos^2 + safeSin^2 = 1`), inverse pairs
  (`safeAcos(safeCos(x)) = x`), finiteness of all outputs.
- `test_chebyshevUtilities_fuzz.cpp`: L1 boundedness, recurrence relation, evaluation at x = +/-1.
- `test_orbitalMotion_fuzz.cpp`: Vis-viva equation, angular momentum conservation, round-trip
  element conversions.

### 9.3 Layered Fuzz Domains

Structure fuzz test input domains in three layers:

**Layer 1 -- Physical Range.** The broadest valid input space reflecting physically realistic
values.

```cpp
// Example from test_oeStateEphem_fuzz.cpp
.WithDomains(
    fuzztest::InRange(1.0, 1e14),           // mu (m^3/s^2)
    fuzztest::InRange(1.0, 1e14),           // r_p (m)
    fuzztest::InRange(0.0, 0.99),           // eccentricity
    fuzztest::InRange(0.0, M_PI),           // inclination
    fuzztest::InRange(0.0, 2.0 * M_PI),    // RAAN
    ...);
```

**Layer 2 -- Edge-Case Bands.** Narrow ranges around known trouble spots.

```cpp
// Example: targeting specific eccentricity regimes
fuzztest::OneOf(
    fuzztest::InRange(0.0, 0.01),     // near-circular
    fuzztest::InRange(0.95, 0.999),   // near-parabolic
    fuzztest::InRange(0.4, 0.6));     // moderate
```

**Layer 3 -- Singularity Stress Tests.** Inputs at or near mathematical singularities.

```cpp
// Example: rectilinear orbit (h = 0 singularity)
fuzztest::InRange(0.9, 0.9999);  // eccentricity approaching parabolic
```

### 9.4 Writing a New Fuzz Test: Principles

1. **Identify mathematical properties.** What invariants, conservation laws, round-trip identities,
   or bounds does the function satisfy?
2. **Choose a strategy.** If an independent reference implementation is feasible, use double-reference
   comparison (Section 9.2.1). Otherwise, use property-based checking (Section 9.2.2).
3. **Define Layer 1 domains** from physical constraints. What are the valid input ranges for this
   algorithm's mission context?
4. **Identify singularities** for Layers 2 and 3. Review Section 5 for which numerical patterns
   apply to your algorithm.
5. **Derive tolerances** from Section 7 methodology. Count operations, multiply by epsilon, add
   margin.
6. **Assert finiteness** for ALL outputs. Every fuzz test should include
   `ASSERT_TRUE(std::isfinite(...))` for every output value -- NaN/Inf propagation is always a bug.

---

## 10. Compile-Time and Runtime Checks

[//]: # (### 10.1 Compile-Time Assertions)

[//]: # ()
[//]: # (Include the following in a common header or in each algorithm's translation unit:)

[//]: # ()
[//]: # (```cpp)

[//]: # (static_assert&#40;sizeof&#40;float&#41; == 4, "float must be 32-bit IEEE 754"&#41;;)

[//]: # (static_assert&#40;sizeof&#40;double&#41; == 8, "double must be 64-bit IEEE 754"&#41;;)

[//]: # (static_assert&#40;std::numeric_limits<float>::is_iec559, "float must be IEEE 754 compliant"&#41;;)

[//]: # (static_assert&#40;std::numeric_limits<double>::is_iec559, "double must be IEEE 754 compliant"&#41;;)

[//]: # (```)

[//]: # ()
[//]: # (These are zero-cost guarantees that the precision assumptions in this document hold on the target.)

### 10.1 Runtime NaN/Inf Detection

All algorithm `update()` or `run()` outputs should be checked for finiteness before writing to
message payloads:

```cpp
// Debug-only finiteness check
assert(std::isfinite(output.sigma_BR[0]));
assert(std::isfinite(output.sigma_BR[1]));
assert(std::isfinite(output.sigma_BR[2]));
```

These checks should be enabled in debug and test builds. For flight builds, they may be compiled out
via `NDEBUG` if the overhead is unacceptable, but the preference is to keep them active.

Reference: The safeMath wrappers already include internal NaN/Inf guards. Runtime output checks
catch cases where NaN/Inf enters through a path that does not go through safeMath.

### 10.2 Input Validation

Validate algorithm inputs at entry points using domain-appropriate checks. Existing patterns to
follow:

- `validDcmCheck.h`: Checks orthogonality (`dcm^T * dcm ~ I` with `1e-5F` tolerance) and
  determinant (~1.0).
- `validInertiaCheck.h`: Checks symmetry, positive eigenvalues (> `1e-6`), and triangle inequality
  on principal moments.

New algorithms should implement similar input validation for their specific domains.

---

## 11. Static Analysis Compliance

### 11.1 Current Static Analysis Configuration

This project uses clang-tidy (`.clang-tidy`) with CERT C++ rules and cppcoreguidelines. The
relevant enabled checks include:

- **`cert-flp30-c`**: Warns against using floating-point variables as loop counters.
- **`cppcoreguidelines-narrowing-conversions`**: Flags implicit narrowing, including double-to-float.
- **`cppcoreguidelines-pro-type-*`**: Type safety rules that complement explicit casting
  requirements.

### 11.2 Precision-Relevant Rules and Practices

| Static Analysis Rule / Best Practice | Precision Guideline Mapping |
|--------------------------------------|-----------------------------|
| Document floating-point arithmetic use | This document (Section 3) |
| No implicit widening or narrowing | Section 3.4: explicit `static_cast<float>()` + comment |
| No floating-point equality comparisons | Use `isApprox()` with explicit tolerance, or EXPECT_NEAR in tests |
| Floating-point standard compliance | Section 10.1: `static_assert(is_iec559)` |
| No FP loop counters | Use integer loop counters with float conversion inside the loop |

### 11.3 Accepted Deviations

The safeMath wrappers in `safeMath.h` intentionally compare against `0.0F` using `==`:

```cpp
// safeAtan2f: checking for exact zero is the correct domain guard
if (y == 0.0F && x == 0.0F) { return 0.0F; }
```

This is an intentional deviation. Checking for exact zero is the correct guard for the `atan2(0, 0)`
undefined case. The comparison is against a literal zero, not a computed quantity, so the usual
concerns about floating-point equality do not apply.

---

## 12. Pre-Implementation Checklist

Before submitting a new algorithm for code review, verify each item:

- [ ] **Precision choice documented.** Each variable's type (`float` vs `double`) is justified.
      Default is `float` unless Section 3.2 applies. Document in the algorithm header file.

- [ ] **Singularities identified.** All division operations, trigonometric calls, and norm-based
      branches are listed with their singularity conditions and guards.

- [ ] **safeMath usage verified.** No direct calls to `sinf`, `cosf`, `acosf`, `asinf`, `atanf`,
      `atan2f`, `sqrtf`, `tanf`, `sinhf`, `coshf`, `tanhf`, `atanhf` or their double equivalents.
      All go through safeMath wrappers.

- [ ] **Tolerance derivation shown.** Every comparison tolerance and convergence criterion has a
      derivation per Section 7 methodology (operation count * epsilon * margin).

- [ ] **Input validation implemented.** Bounds checking, NaN rejection, and domain validity checks
      at algorithm entry points. Follow `validDcmCheck.h` / `validInertiaCheck.h` patterns.

- [ ] **Output validation.** All outputs checked for finiteness (`std::isfinite`) before writing to
      message payloads.

- [ ] **Mixed-precision boundaries marked.** Every `double`-to-`float` transition uses
      `static_cast<float>()` with a comment. All float literals use the `F` suffix.

- [ ] **Fuzz test plan.** Identified which strategy (double-reference or property-based) applies.
      Defined three domain layers (physical range, edge-case bands, singularity stress). Derived
      tolerances from methodology.

- [ ] **Precision assumptions documented.** The algorithm header file states the precision
      assumptions (e.g., "MRP norm must be < 1", "eccentricity must be < 0.99 for float32
      precision").

- [ ] **Eigen usage reviewed.** No implicit type mixing, appropriate `isApprox` tolerances
      specified explicitly (not relying on defaults), awareness of operations from Section 6.3.

---

## Appendix A: Compiler Flags

The following compiler flags enforce precision discipline and should be enabled for all builds:

| Flag | Purpose | Notes |
|------|---------|-------|
| `-Wdouble-promotion` | Warns when a `float` is implicitly promoted to `double` | Catches missing `F` suffix on literals and accidental mixed-precision expressions |
| `-Wfloat-conversion` | Warns on implicit conversions that may lose floating-point precision | Catches double-to-float narrowing without explicit cast |
| `-Wfloat-equal` | Warns on direct `==` comparison of floating-point values | Expected to trigger on intentional comparisons in safeMath (suppress with comment) |
| `-Wconversion` | General conversion warnings including float/double narrowing | Broader than `-Wfloat-conversion`, catches integer-to-float issues too |

These flags should be enabled in both the native Linux build and the RISC-V cross-compilation
toolchain (`riscv32-toolchain.cmake`).
