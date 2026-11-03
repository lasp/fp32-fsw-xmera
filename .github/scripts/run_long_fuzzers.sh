#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 <build_dir> <log_dir> [--fuzz-for <duration>] [--corpus <path>]" \
       "[--shard-index <i>] [--shard-count <n>]" >&2
}

if [[ $# -lt 2 ]]; then
  usage
  exit 2
fi

BUILD_DIR="$1"
LOG_DIR="$2"
shift 2

FUZZ_DURATION="120s"
CORPUS_DIR=""
SHARD_INDEX=0
SHARD_COUNT=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fuzz-for)
      [[ $# -ge 2 ]] || { echo "--fuzz-for requires a value" >&2; exit 2; }
      FUZZ_DURATION="$2"
      shift 2
      ;;
    --corpus)
      [[ $# -ge 2 ]] || { echo "--corpus requires a path" >&2; exit 2; }
      CORPUS_DIR="$2"
      shift 2
      ;;
    --shard-index)
      [[ $# -ge 2 ]] || { echo "--shard-index requires a value" >&2; exit 2; }
      SHARD_INDEX="$2"
      shift 2
      ;;
    --shard-count)
      [[ $# -ge 2 ]] || { echo "--shard-count requires a value" >&2; exit 2; }
      SHARD_COUNT="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if ! [[ "$SHARD_COUNT" =~ ^[1-9][0-9]*$ && "$SHARD_INDEX" =~ ^[0-9]+$ ]] \
   || (( SHARD_INDEX >= SHARD_COUNT )); then
  echo "Need 0 <= --shard-index < --shard-count (got ${SHARD_INDEX} / ${SHARD_COUNT})" >&2
  exit 2
fi

mkdir -p "$LOG_DIR"
if [[ -n "$CORPUS_DIR" ]]; then
  mkdir -p "$CORPUS_DIR"
fi

# Discover fuzz binaries via CTest labels
FUZZ_BINS=$(BUILD_DIR="$BUILD_DIR" python3 - <<'PY'
import json, os, subprocess
build_dir = os.environ["BUILD_DIR"]
proc = subprocess.run(
    ["ctest", "--show-only=json-v1", "-L", "fuzz"],
    capture_output=True, text=True, check=True,
    cwd=build_dir
)
data = json.loads(proc.stdout or "{}")
seen = []
for test in data.get("tests", []):
    cmd = test.get("command", [])
    if cmd:
        exe = cmd[0]
        if exe not in seen:
            seen.append(exe)
print("\n".join(sorted(seen, key=os.path.basename)))
PY
)

if [[ -z "$FUZZ_BINS" ]]; then
  echo "No fuzz executables discovered via CTest labels." >&2
  exit 1
fi

# Enumerate every (binary, FUZZ_TEST) pair in a stable order. --list_fuzz_tests
# prints one line per test in the form "[*] Fuzz test: <Suite.Name>" and exits.
all_bins=()
all_tests=()
while IFS= read -r fuzz_bin; do
  [[ -z "$fuzz_bin" ]] && continue
  mapfile -t fuzz_tests < <("$fuzz_bin" --list_fuzz_tests 2>/dev/null \
    | sed -n 's/^\[\*\] Fuzz test: //p' | LC_ALL=C sort)

  if [[ ${#fuzz_tests[@]} -eq 0 ]]; then
    echo "WARNING: No FUZZ_TESTs discovered in $(basename "$fuzz_bin"); skipping." >&2
    continue
  fi

  for test_name in "${fuzz_tests[@]}"; do
    all_bins+=("$fuzz_bin")
    all_tests+=("$test_name")
  done
done <<< "$FUZZ_BINS"

# Shard by test, not by binary: the number of tests per binary varies a lot.
# The manifest lists what this shard is meant to run, so the report can tell a
# test that never started from one that finished.
manifest="${LOG_DIR}/manifest-shard-${SHARD_INDEX}.txt"
: > "$manifest"
shard_bins=()
shard_tests=()
for i in "${!all_tests[@]}"; do
  if (( i % SHARD_COUNT == SHARD_INDEX )); then
    shard_bins+=("${all_bins[$i]}")
    shard_tests+=("${all_tests[$i]}")
    echo "$(basename "${all_bins[$i]}").${all_tests[$i]}" >> "$manifest"
  fi
done

echo ">>> Shard ${SHARD_INDEX}/${SHARD_COUNT}: ${#shard_tests[@]} of ${#all_tests[@]} fuzz tests"

CRASH_COUNT=0

for i in "${!shard_tests[@]}"; do
  fuzz_bin="${shard_bins[$i]}"
  test_name="${shard_tests[$i]}"
  fuzz_name="$(basename "$fuzz_bin")"

  echo ">>> Running long fuzz session for ${fuzz_name} :: ${test_name}"
  cmd=("$fuzz_bin" "--fuzz=${test_name}")
  if [[ -n "$FUZZ_DURATION" ]]; then
    cmd+=("--fuzz_for=${FUZZ_DURATION}")
  fi
  env_vars=()
  if [[ -n "$CORPUS_DIR" ]]; then
    fuzz_corpus_dir="${CORPUS_DIR}/${fuzz_name}/${test_name}"
    mkdir -p "$fuzz_corpus_dir"
    env_vars+=(
      "FUZZTEST_TESTSUITE_OUT_DIR=${fuzz_corpus_dir}"
      "FUZZTEST_TESTSUITE_IN_DIR=${fuzz_corpus_dir}"
    )
  fi
  log_file="${LOG_DIR}/${fuzz_name}.${test_name}.log"
  if env "${env_vars[@]}" "${cmd[@]}" 2>&1 | tee "$log_file"; then
    echo ">>> ${fuzz_name} :: ${test_name}: OK"
  else
    echo ">>> ${fuzz_name} :: ${test_name}: FAILED (exit $?)"
    CRASH_COUNT=$((CRASH_COUNT + 1))
  fi

  # FuzzTest skips corpus files it cannot parse, which happens once a test's
  # input domain changes. Nothing else removes them, so they pile up in the
  # cache. Delete the ones it reported, but only inside this test's corpus.
  if [[ -n "$CORPUS_DIR" ]]; then
    pruned=0
    while IFS= read -r stale; do
      if [[ "$stale" == "${fuzz_corpus_dir}/"* && -f "$stale" ]]; then
        rm -f -- "$stale"
        pruned=$((pruned + 1))
      fi
    done < <(sed -n 's/^\[!\] Invalid input file \([^:]*\): .*$/\1/p' "$log_file")
    if (( pruned > 0 )); then
      echo ">>> ${fuzz_name} :: ${test_name}: pruned ${pruned} stale corpus file(s)"
    fi
  fi
done

if [[ $CRASH_COUNT -gt 0 ]]; then
  echo "ERROR: ${CRASH_COUNT} fuzz target(s) reported crashes." >&2
  exit 1
fi
