#!/usr/bin/env bash
# Replay a corpus written by run_long_fuzzers.sh: for each
# <corpus>/<binary>/<Suite.Test> directory, run that one test once on every
# input in it. FUZZTEST_REPLAY is FuzzTest's own replay switch and works in
# both the fuzzing (Clang) and the unit-test (GCC) build.
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <build_dir> <corpus_dir>" >&2
  exit 2
fi

BUILD_DIR="$1"
CORPUS_DIR="$2"

if [[ ! -d "$CORPUS_DIR" ]]; then
  echo "No corpus at ${CORPUS_DIR}; nothing to replay."
  exit 0
fi

# Map binary basename -> path, from the same CTest labels the fuzz script uses.
declare -A BIN_PATH
while IFS= read -r exe; do
  [[ -n "$exe" ]] && BIN_PATH["$(basename "$exe")"]="$exe"
done < <(BUILD_DIR="$BUILD_DIR" python3 - <<'PY'
import json, os, subprocess
proc = subprocess.run(
    ["ctest", "--show-only=json-v1", "-L", "fuzz"],
    capture_output=True, text=True, check=True, cwd=os.environ["BUILD_DIR"],
)
seen = set()
for test in json.loads(proc.stdout or "{}").get("tests", []):
    cmd = test.get("command", [])
    if cmd and cmd[0] not in seen:
        seen.add(cmd[0])
        print(cmd[0])
PY
)

if [[ ${#BIN_PATH[@]} -eq 0 ]]; then
  echo "No fuzz executables discovered via CTest labels." >&2
  exit 1
fi

replayed=0
inputs=0
skipped=0
failed=()

while IFS= read -r test_dir; do
  test_name="$(basename "$test_dir")"
  fuzz_name="$(basename "$(dirname "$test_dir")")"
  count=$(find "$test_dir" -maxdepth 1 -type f | wc -l)
  (( count > 0 )) || continue

  fuzz_bin="${BIN_PATH[$fuzz_name]:-}"
  if [[ -z "$fuzz_bin" ]]; then
    echo "::warning::No fuzz binary ${fuzz_name} for corpus ${fuzz_name}/${test_name}; skipped."
    skipped=$((skipped + 1))
    continue
  fi

  echo ">>> Replaying ${count} input(s) for ${fuzz_name} :: ${test_name}"
  if ! out=$(FUZZTEST_REPLAY="$test_dir" "$fuzz_bin" "--gtest_filter=${test_name}" 2>&1); then
    echo "$out"
    failed+=("${fuzz_name} :: ${test_name}")
    continue
  fi
  # A renamed or deleted test leaves its corpus behind. gtest then selects
  # nothing and passes, so say so instead of counting it as replayed.
  if grep -q '^\[==========\] Running 0 tests' <<< "$out"; then
    echo "::warning::${fuzz_name} has no test ${test_name}; its corpus was not replayed."
    skipped=$((skipped + 1))
    continue
  fi
  replayed=$((replayed + 1))
  inputs=$((inputs + count))
done < <(find "$CORPUS_DIR" -mindepth 2 -maxdepth 2 -type d | LC_ALL=C sort)

echo ">>> Replayed ${inputs} input(s) across ${replayed} fuzz test(s); ${skipped} skipped, ${#failed[@]} failed."
if (( ${#failed[@]} > 0 )); then
  printf 'FAILED: %s\n' "${failed[@]}" >&2
  exit 1
fi
