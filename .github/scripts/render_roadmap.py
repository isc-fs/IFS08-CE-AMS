#!/usr/bin/env python3
"""Render ROADMAP.md from .github/roadmap.yaml.

Loads the declarative roadmap, looks up each branch's tracking issue
state via `gh issue list`, and writes a fresh ROADMAP.md with status
badges, a phase table and a mermaid gitGraph.

Requires: pyyaml, and the `gh` CLI authenticated via GH_TOKEN /
GITHUB_TOKEN. Writes to ROADMAP.md in the repository root.
"""
from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
from typing import Optional

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
YAML_PATH = ROOT / ".github" / "roadmap.yaml"
OUT_PATH = ROOT / "ROADMAP.md"

BADGE = {
    "closed":   "✅ done",
    "open":     "🔄 active",
    "deferred": "⏸ deferred",
    None:       "🔜 planned",
}


def tracking_issue_states(repo: str) -> dict[str, str]:
    """Return {branch_name: 'open'|'closed'} for every tracking issue.

    A tracking issue is one with label `feat` or `fix` whose title begins
    with `[<branch>]`. `gh issue list` only returns real issues — pull
    requests are already filtered out by the CLI.
    """
    results: dict[str, str] = {}
    for label in ("feat", "fix"):
        raw = subprocess.run(
            [
                "gh", "issue", "list",
                "--repo", repo,
                "--label", label,
                "--state", "all",
                "--limit", "500",
                "--json", "title,state",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        for issue in json.loads(raw):
            title = issue.get("title", "")
            if not title.startswith("["):
                continue
            close = title.find("]")
            if close <= 1:
                continue
            branch = title[1:close]
            results[branch] = issue["state"].lower()
    return results


def badge(state: Optional[str]) -> str:
    return BADGE.get(state, BADGE[None])


def render_table(phases: list, sidequests: list, states: dict[str, str]) -> str:
    lines = [
        "| Phase | Title | Branches | Milestone tag |",
        "|:---:|---|---|---|",
    ]
    for phase in phases:
        is_deferred = bool(phase.get("deferred"))
        items = []
        for b in phase["branches"]:
            # Deferred phases override per-branch tracking state so the
            # badge column reads `⏸ deferred` uniformly — the intent
            # ("this work is paused") matters more than the state of any
            # individual tracking issue.
            state = "deferred" if is_deferred else states.get(b["name"])
            items.append(f"{badge(state)} `{b['name']}`")
        title = phase["title"]
        if is_deferred:
            title += " _(deferred)_"
        row_note = phase.get("note")
        if row_note:
            title += f"<br><sub>{row_note}</sub>"
        lines.append(
            f"| {phase['number']} | {title} | "
            f"{' · '.join(items)} | `{phase.get('tag', '—')}` |"
        )
    for sq in sidequests or []:
        items = [
            f"{badge(states.get(b))} `{b}`" for b in sq["branches"]
        ]
        lines.append(
            f"| — | {sq['title']} _(sidequest)_ | "
            f"{' · '.join(items)} | — |"
        )
    return "\n".join(lines)


def _quote(text: str) -> str:
    """Sanitize a string so it is safe inside a mermaid double-quoted id."""
    return (
        text.replace("\\", "/")
            .replace('"', "'")
            .replace("\n", " ")
            .strip()
    )


def render_mermaid(phases: list, states: dict[str, str]) -> str:
    out: list[str] = [
        "```mermaid",
        "%%{init: { 'gitGraph': { 'mainBranchName': 'main', 'showCommitLabel': true }}}%%",
        "gitGraph",
        '    commit id: "import"',
        '    commit id: "baseline" tag: "v0.1.0-baseline"',
        "    branch dev",
        "    checkout dev",
        "",
    ]
    for phase in phases:
        # Deferred phases don't belong in the branch-flow diagram — they
        # haven't been cut and aren't on the trajectory to the next tag.
        # The table above already makes the deferred status explicit.
        if phase.get("deferred"):
            continue
        out.append(f"    %% Phase {phase['number']} — {phase['title']}")
        for b in phase["branches"]:
            name = b["name"]
            scope = _quote(b["scope"])
            state = states.get(name)
            marker = {"closed": "✔ ", "open": "… "}.get(state, "○ ")
            out.append(f"    branch {name}")
            out.append(f'    commit id: "{marker}{scope}"')
            out.append("    checkout dev")
            out.append(f"    merge {name}")
        tag = phase.get("tag")
        if tag:
            out.append("    checkout main")
            out.append(f'    merge dev tag: "{tag}"')
            out.append("    checkout dev")
        out.append("")
    out.append("```")
    return "\n".join(out)


AUTOGEN_NOTE = """<!--
  Auto-generated from .github/roadmap.yaml by
  .github/scripts/render_roadmap.py. Do not edit this file directly —
  changes will be overwritten on the next run of the `Render ROADMAP`
  workflow. Edit the YAML or close / reopen a tracking issue instead.
-->"""

# Fallbacks used when roadmap.yaml doesn't provide its own `title` and
# `intro`. Kept deliberately generic so repos forked from the plantilla
# template get sensible placeholder content until they customise.
DEFAULT_TITLE = "Project roadmap"
DEFAULT_INTRO = (
    "Phased delivery plan for this repository. Each phase is a cluster "
    "of feat branches cut from `dev`; a milestone tag on `main` closes "
    "the phase once every branch in it has merged. Branch status badges "
    "(✅ / 🔄 / 🔜) are derived from each branch's tracking issue state "
    "in GitHub Issues."
)

FOOTER = """---

## How this file is maintained

- **Source of truth**: [`.github/roadmap.yaml`](.github/roadmap.yaml)
- **Renderer**: [`.github/scripts/render_roadmap.py`](.github/scripts/render_roadmap.py)
- **Workflow**: [`.github/workflows/roadmap.yml`](.github/workflows/roadmap.yml)

The workflow runs on every push to `dev`. If any branch status changed
(a tracking issue was closed, the YAML was edited, a new branch was
added) it regenerates this file and auto-commits with the message
`chore: regenerate ROADMAP.md [skip ci]`.
"""


def main() -> int:
    repo = os.environ.get("GITHUB_REPOSITORY")
    if not repo:
        print("ERROR: GITHUB_REPOSITORY environment variable is required.",
              file=sys.stderr)
        return 2

    plan = yaml.safe_load(YAML_PATH.read_text())
    phases = plan.get("phases", [])
    sidequests = plan.get("sidequests", [])
    title = plan.get("title") or DEFAULT_TITLE
    intro = (plan.get("intro") or DEFAULT_INTRO).strip()

    states = tracking_issue_states(repo)

    body = "\n\n".join([
        AUTOGEN_NOTE,
        f"# {title}",
        intro,
        "## Phase summary",
        render_table(phases, sidequests, states),
        "## Branch diagram",
        render_mermaid(phases, states),
        FOOTER.rstrip(),
    ]) + "\n"

    OUT_PATH.write_text(body)
    print(f"rendered {OUT_PATH.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
