#!/usr/bin/env bash
# check_guardrails.sh — the regression gate for core/HAL refactor work.
#
# Pins the three canonical boots BYTE-IDENTICAL (exact lcd.pgm md5) plus the
# unit suites. Run it before AND after any change to the bus dispatch, the
# peripheral models, the model profiles, or the C54x core. Any drift = stop.
#
#   tools/check_guardrails.sh            # all gates (two 250M boots + 30M cosim + make test)
#   GATES="3310 5110" tools/check_guardrails.sh   # subset while iterating
#
# Baselines pinned 2026-06-12 on commit 183d22f (see git log for context):
#   3310  Factory Reset NR1 v5.79, plain boot_trace 250M, 0 resets -> standby+clock
#   3410  NHM-2 v5.46 assembled,   plain boot_trace 250M, 0 resets -> standby
#   5110  NSE-1 v5.30 A, DSP54_COSIM=1 30M (check_5110_boot.sh)    -> Security code
#
# The boots run honest (no RESET_RECOVER): a caught firmware self-reset fails
# the gate even if the final screen happens to match.
#
# Exit code: 0 = all gates pass, 1 = drift/failure, 2 = harness error.

set -uo pipefail
cd "$(dirname "$0")/.."

GATES="${GATES:-test 3310 3410 5110}"
FAILED=0

# Exact-match boot gate: plain boot_trace, no resets allowed, pinned lcd md5.
boot_gate() { # name fw steps want_md5 [extra env...]
    local name="$1" fw="$2" steps="$3" want="$4"; shift 4
    local log="/tmp/guard_${name}.log"
    if [ ! -f "$fw" ]; then echo "GUARD $name: HARNESS ERROR — missing firmware: $fw"; return 2; fi
    env "$@" ./build/dct3_boot_trace "$fw" "$steps" > "$log" 2>&1
    local rc=$?
    if [ $rc -ne 0 ]; then
        echo "GUARD $name: FAIL — emulator rc=$rc (log: $log)"; return 1
    fi
    if grep -qE "^\[reset\] CATCH reason=" "$log"; then
        echo "GUARD $name: FAIL — $(grep -cE '^\[reset\] CATCH reason=' "$log") firmware reset(s): $(grep -m1 -E '^\[reset\] CATCH reason=' "$log") (log: $log)"
        return 1
    fi
    local got; got=$(md5sum build/lcd.pgm | awk '{print $1}')
    if [ "$got" != "$want" ]; then
        echo "GUARD $name: FAIL — lcd drift: got $got want $want (log: $log, screen: build/lcd.pgm)"
        return 1
    fi
    echo "GUARD $name: PASS ($steps steps, lcd $got)"
    return 0
}

for g in $GATES; do
    case "$g" in
    test)
        if make -s test > /tmp/guard_test.log 2>&1; then
            echo "GUARD test: PASS ($(grep -cE 'passed, 0 failed' /tmp/guard_test.log) suites green)"
        else
            echo "GUARD test: FAIL (see /tmp/guard_test.log)"; FAILED=1
        fi ;;
    3310)
        boot_gate 3310 "firmware/Factory Reset 3310 NR1 v5.79.fls" 250000000 \
                  82610e5e05036d7764afe419c5a48f1d || FAILED=1 ;;
    3410)
        boot_gate 3410 "firmware/Nokia 3410 NHM-2 v5.46 (assembled).fls" 250000000 \
                  5a284ea4d2cfe72792c2c696c598b51f || FAILED=1 ;;
    5110)
        # The cosim oracle script classifies the screen; we additionally pin the
        # exact Security-code hash so ANY pixel drift fails, not just CONTACT
        # SERVICE/blank. (4921792d = the documented canonical-state oracle.)
        if tools/check_5110_boot.sh > /tmp/guard_5110_oracle.log 2>&1; then
            got=$(md5sum build/lcd.pgm | awk '{print $1}')
            if [ "$got" = "4921792d5d96cd38f98a83d1e436a5af" ]; then
                echo "GUARD 5110: PASS (cosim 30M, lcd $got)"
            else
                echo "GUARD 5110: FAIL — oracle passed but lcd drift: got $got want 4921792d5d96cd38f98a83d1e436a5af"
                FAILED=1
            fi
        else
            echo "GUARD 5110: FAIL — $(tail -1 /tmp/guard_5110_oracle.log)"; FAILED=1
        fi ;;
    *) echo "GUARD: unknown gate '$g' (have: test 3310 3410 5110)"; FAILED=1 ;;
    esac
done

if [ $FAILED -ne 0 ]; then echo "GUARDRAILS: FAIL"; exit 1; fi
echo "GUARDRAILS: ALL PASS"
exit 0
