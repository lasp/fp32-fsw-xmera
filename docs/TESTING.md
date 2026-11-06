# Organizing Tests

## Test Files
- a test file is named `test_<module_name/name_of_file>.cpp`
- any fuzz tests are in a file(s) named `test_<module_name/name_of_file>_fuzz.cpp`
- any utility/helper functions used across tests for a module are placed in `<module_name>Helpers.cpp`
- a pytest python test file is named `test_<module_name/name_of_file>.py`

## Naming Tests
- Prefer longer expressive names over shorter names. This assists such that when a test
fails the specific case is easily read from the output.
- A property (fuzz) test should be distinguished by its name as `Property<remainder_of_test_name>`

# Running the C++ Tests

**Adamant-Xmera Flight Software Algorithms (fp32-fsw-xmera)**

This document describes how the C++ host tests are organized and how to run subsets of
them with CTest. Each algorithm keeps its host gtest (and FuzzTest) suites in an `_tests/`
directory beside the code; tests are grouped with CTest **labels** so the slow and optional
suites can be included or excluded explicitly.

---

## 1. Label vocabulary

| Label                 | What it is                                                        | Built by default? | Run in PR CI?     |
|-----------------------|-------------------------------------------------------------------|-------------------|-------------------|
| *(unlabeled)*         | Fast unit / validation tests — the common case                    | yes               | yes               |
| `fuzz` + `fuzz-smoke` | FuzzTest targets (both labels applied together)                   | only when fuzzing is enabled | smoke subset only |
| `exhaustive`          | Long brute-force scans (currently only `utilities/fsw/_tests`)    | yes               | no                |

`fuzz`/`fuzz-smoke` are used across most algorithm `_tests/` directories. `exhaustive` is
currently specific to `algorithms/utilities/fsw/_tests` — see that directory's `README.md`.

> CTest's `-L` (label include) and `-LE` (label exclude) take **regexes**, and a bare `ctest` runs
> *everything configured in the build* — it does not skip `exhaustive`. Exclude long test groups
> explicitly.

## 2. Running tests

Run CTest from the algorithms build directory, e.g. `build/fp32-fsw-xmera/algorithms`:

```bash
# Everything that is configured
ctest --output-on-failure

# Fast suites only — skip the long test groups
ctest --output-on-failure -LE "exhaustive|fuzz"

# A single module's tests
ctest --output-on-failure -R <moduleName>

# Just list matching tests without running them
ctest -N -R <moduleName>
```

## 3. Fuzz tests

The algorithm fuzz tests are only built when this repo is being compiled as a external modules
root in Xmera. The `*_fuzz` targets are gated behind the CMake option `XMERA_ENABLE_FUZZTESTS`
(defined in `src/cmake/XmeraGoogleTest.cmake`, **OFF** by default) and are not built otherwise.
Enable them with a preset (`src/CMakePresets.json`):

```bash
cmake --preset fuzz-smoke-test    # builds fuzz targets; short smoke runs
cmake --preset fuzz-test          # adds FUZZTEST_FUZZING_MODE for long corpus runs

# then, from the algorithms build dir:
ctest --output-on-failure -L fuzz-smoke    # quick fuzz smoke pass
ctest --output-on-failure -L fuzz          # all fuzz targets
```

## 4. What CI runs

- **Pull request** (`.github/workflows/pull-request.yml`): configures `fuzz-smoke-test`, then
  `ctest --output-on-failure -LE exhaustive`. The fuzz targets therefore *do* run on every PR,
  in unit-test mode; the brute-force scans do not, because under `--coverage` their hit counts
  overflow gcov. The Linux leg builds instrumented and reports coverage.
- **Daily fuzzing** (`.github/workflows/daily-fuzz.yml`): every job configures `fuzz-test` with
  Clang.
  - `fuzz-smoke` runs the fuzz targets in unit-test mode, replays every input in the cached
    corpus with `FUZZTEST_REPLAY`, and runs the `exhaustive` scans.
  - The `fuzz-shard` matrix fuzzes each `FUZZ_TEST` for `fuzz_duration` (default 60 s). The
    tests are dealt round-robin across the shards, because one job cannot fuzz all of them
    inside the 6 h job limit. When the test count grows, add shards to the matrix.
  - After each session, the shard deletes the corpus inputs that FuzzTest rejected as invalid.
    These are inputs from a domain that has since changed.
  - `fuzz-merge` replaces each fuzzed test's corpus in the cache and writes the report. A test
    is `CRASH`, `INCOMPLETE` (no gtest PASSED line, for example because the job was killed),
    `NOT RUN`, or `OK`. Any status other than `OK` fails the job. It also deletes the corpus of
    any test that no shard lists, which is what a renamed or deleted `FUZZ_TEST` leaves behind.

The pull request workflow publishes coverage artifacts. See [`COVERAGE.md`](COVERAGE.md) for what
the reports contain and how to read the line and branch numbers.
