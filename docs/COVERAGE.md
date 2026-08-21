# Reading the Coverage Reports

CI publishes gcov-based coverage for the code in `algorithms/`. This page explains what the
reports contain, and — more importantly — what the numbers do and do not mean.

## 1. Which report comes from where

| Artifact | Produced by | What ran |
|----------|-------------|----------|
| `external-modules-coverage` | `pull-request.yml`, Linux leg | ctest (unit tests + fuzz targets in unit-test mode) followed by pytest, all against one instrumented build |
| `daily-fuzz-coverage` | `daily-fuzz.yml`, `fuzz-coverage` job | the accumulated fuzz corpus replayed, plus the `exhaustive` brute-force scans |

Each artifact holds an HTML report (`--html-details`), a plain-text line summary, a plain-text
branch summary, a machine-readable JSON report, and a completeness table.

The macOS leg of the pull request job is not instrumented, so it contributes nothing. Neither
report is a gate: nothing fails on a coverage number.

## 2. Read the completeness table first

gcovr only reports a file if that file produced profile data. A source that was never compiled,
or compiled but never executed, is **absent from the report** rather than listed at 0%. The
percentage is therefore an average over whatever happened to be measured, and it goes *up* when
code falls out of the build.

`external-modules-completeness.txt` (from `.github/scripts/audit_coverage_completeness.sh`)
exists to make that visible. It classifies every non-test translation unit under `algorithms/`
as compiled, executed, reported, or missing, and lists the missing ones by name.

A high percentage over a short file list means less than a lower percentage over a complete one.
Two entries are expected there today and are not regressions:

- the `*Algorithm_c.cpp` shims, which the Xmera build mostly does not compile. They are built
  into the freestanding `libgncAlgorithms.a` and tested downstream through the Ada bindings in
  `adamant-xmera-components`, under a different coverage tool entirely.
- anything with no test at all, which the table names explicitly.

## 3. What gcov counts as a branch

A "branch" is a conditional jump the compiler emitted. It is **not** an `if` in the source, and
the two are far apart in this codebase.

Concretely, from a real report on this repo: line 522 of
`algorithms/utilities/fsw/rigidBodyKinematics.hpp` is a single statement,

```cpp
tilde << ScalarT(0), -vector(2), vector(1), vector(2), ScalarT(0), -vector(0), -vector(1), vector(0), ScalarT(0);
```

and it carries **56 branches**, of which 14 were taken. There is no decision in that line at
all. The branches belong to the Eigen comma-initializer machinery inlined into it.

Two consequences follow, and they matter more than any single percentage.

**Inlined library code counts.** The gcovr filter excludes Eigen's own headers from the report,
but it cannot exclude Eigen code that was inlined into ours. On the header-heavy parts of this
repo — `filteringCore`, `utilities/fsw` — inlined machinery dominates the branch denominator.
Short-circuit `&&` and `||` add to it too, so one line can hold several genuine branches on top
of that.

**Templates are counted once per instantiation.** gcov emits a separate record per instantiation
and gcovr keeps them separate. In the same header, 124 lines have two records apiece, one for
the `float` instantiation and one for `double`. Line 522 appears twice, once with 28 branches
and once with 56, 14 taken in each. So a header's branch percentage aggregates across
instantiations: a branch shown as taken may be taken by only one of them, and a header at a low
branch percentage may be fully exercised for the instantiation you care about.

Because the denominator is a property of the compiler and the optimisation level, a branch
percentage is only comparable against another run built the same way. The pull request job is
GCC 13 at `-O0`. Do not compare it against a local Clang build, against the uninstrumented
macOS leg, or against the downstream Ada report.

## 4. How to use the reports

Do not chase the aggregate — on template-heavy code it is largely a measure of how much library
machinery got inlined.

Open the HTML report instead, and look for never-taken branches on lines that are real
source-level decisions. Sort each one into:

- **untested validation or error handling** — actionable, write a test;
- **a defensive path unreachable through the public API** — leave it, and say why in a comment;
- **compiler or library machinery** — ignore it.

Only the first category is work. The line report is the better guide to whether a file is
tested at all; the branch report is the better guide to whether its decisions are.

## 5. Branch coverage and the fuzz tests

Branch reach is where fuzzing shows up. The pull request job runs the fuzz targets in unit-test
mode, so some branches are covered there only because a random input happened to reach them —
covered in the report, but not pinned by any deterministic test. If such a branch matters,
promote it to an ordinary gtest case.

Comparing `daily-fuzz-coverage` against `external-modules-coverage` shows what only the
accumulated corpus and the exhaustive scans reach.

## 6. Why there is no threshold

The branch denominator moves with the compiler version, the optimisation level, and how much
library code gets inlined — none of which track test quality. A `--fail-under-branch` gate would
flap on unrelated toolchain changes and teach people to ignore it. The reports are informational
by design, and the completeness table is what catches real regressions.

See [`TESTING.md`](TESTING.md) for what CI runs and how to run the suites locally.
