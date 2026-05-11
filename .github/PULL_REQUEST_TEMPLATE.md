<!--
  Fill in every section. PRs that omit "Closes #N" will not auto-close
  their tracking issue when merged into dev.
-->

## Summary

<!-- 1–3 bullets on what changed and why. -->

-

## Linked issue

Closes #

## Phase / milestone

<!-- Which roadmap phase does this PR belong to? See ROADMAP.md. -->

- Phase:
- Branch: `feat/N` or `fix/N`

## Type of change

- [ ] New functionality (`feat/*`)
- [ ] Bug fix (`fix/*`)
- [ ] Refactor / no behaviour change
- [ ] Documentation only
- [ ] Build / tooling / CI

## Safety impact

<!-- Required for anything touching SafetyTask, relay driver, FSM, or
     the safety predicates. Apply the `safety-critical` label. -->

- [ ] No safety impact
- [ ] Touches safety supervisor / relay driver / FSM → I have updated
      `docs/ARCHITECTURE.md` if invariants changed and applied the
      `safety-critical` label.

## Testing

<!-- What did you run? Tick all that apply. -->

- [ ] Unit tests pass locally (`cmake --build build && ctest`)
- [ ] SIL integration tests pass
- [ ] HIL build green (`/hil-build <subdir>` once available)
- [ ] Bench-tested on hardware
- [ ] On-vehicle (`hardware-required` label)
- [ ] Not testable in this PR — explain:

## Checklist

- [ ] Branch is `feat/<n>` or `fix/<n>` with the next sequential number
- [ ] PR targets `dev` (never `main` directly)
- [ ] Code compiles with no new warnings
- [ ] Added / updated `constexpr` config in `Core/Inc/app/ams_config.hpp`
      if introducing new IDs, thresholds, or periods
- [ ] Updated `docs/CAN_MAP.md` if CAN frames changed
- [ ] Updated `docs/ARCHITECTURE.md` if task layout / synchronisation
      changed
