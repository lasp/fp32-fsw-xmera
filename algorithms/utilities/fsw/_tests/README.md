# `utilities/_tests`

Host-side gtest (and FuzzTest) suites for the header-only helpers in `algorithms/utilities/`
(`validDcm`, `validateInertia`, `chebyshevUtilities`, `safeMath`, `orbitalMotion`,
`freestandingIsFinite`). `CMakeLists.txt` in this directory is the source of truth for what is built
and how each suite is registered.

For the general CTest label vocabulary, fuzz presets, and CI behavior shared across the repo,
see [`docs/TESTING.md`](../../../docs/TESTING.md). This file covers only what is specific to
this directory.

## The `exhaustive` label

`test_freestandingIsFinite` is the one suite here that registers an `exhaustive`-labeled bucket: a
brute-force equivalence proof against the standard-library oracle over **all 2³² float
patterns** plus **50M random doubles**. It is cheap (~5 s total) and runs by default, but can
be selected or skipped on its own:

```bash
# from the algorithms build dir, e.g. build/fp32-fsw-xmera/algorithms
ctest -L exhaustive                              # only the brute-force scans
ctest -R freestandingIsFinite                          # the whole freestandingIsFinite suite
ctest -R freestandingIsFinite --label-exclude exhaustive   # its fast subset only
ctest -R freestandingIsFinite -N                        # list, don't run
```

`exhaustive` is currently unique to this directory; everywhere else the only labels are
`fuzz`/`fuzz-smoke`.

## `test_freestandingIsFinite` registration notes

The suite is registered twice from a single executable (`test_freestandingIsFinite.cpp`):

- a **fast** bucket — the parameterized equivalence-class, boundary, and FP-flag cases; and
- an **`exhaustive`** bucket — the two brute-force scans, given the `exhaustive.` test prefix.

Both registrations pass `NO_PRETTY_VALUES` so CTest keeps the generator-produced case names
(`pos_zero`, `snan`, …) instead of gtest's `# GetParam() = 16-byte object <hex>` parameter
dump (the `FCase`/`DCase` structs have no `operator<<`). The executable defines its own
`main()`, so it links `GTest::gtest` rather than `GTest::gtest_main`. The compile-time
`static_assert` evidence lives in `freestandingIsFinite_static_tests.hpp` and fires during the build.
