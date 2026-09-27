#!/usr/bin/env bash
# Builds a Markdown coverage report from `cargo llvm-cov --json --summary-only` output.
#
# Usage: coverage-table.sh <head.json> [base.json]
# If base.json is missing or empty (e.g. the base branch has no Rust code yet),
# the base column shows "n/a". Only reports; never fails on low coverage.
set -euo pipefail

head_json=$1
base_json=${2:-}
if [[ -z "$base_json" || ! -s "$base_json" ]]; then
  base_json=$(mktemp)
  echo '{"data":[{"totals":{},"files":[]}]}' > "$base_json"
fi

jq -r -n --slurpfile head "$head_json" --slurpfile base "$base_json" '
  def pct: if . == null then "n/a" else (. * 100 | round / 100 | tostring) + "%" end;
  def delta($b; $h):
    if $b == null or $h == null then ""
    else ($h - $b) as $d
      | if ($d | fabs) < 0.005 then "±0"
        elif $d > 0 then "+" + ($d * 100 | round / 100 | tostring)
        else ($d * 100 | round / 100 | tostring) end
    end;
  # Paths are absolute and differ between checkouts; key files by repo-relative path.
  def rel: sub("^.*?/(?<p>[^/]+/src/.*)$"; "\(.p)");
  def files($r): [$r.data[0].files[] | {key: (.filename | rel), value: .summary.lines.percent}] | from_entries;

  $head[0] as $h | $base[0] as $b |
  "## Coverage report",
  "",
  "| Metric | Base | This PR | Change |",
  "|---|---|---|---|",
  ( ["lines", "functions", "regions"][] as $m
    | ($b.data[0].totals[$m].percent) as $bp
    | ($h.data[0].totals[$m].percent) as $hp
    | "| \($m) | \($bp | pct) | \($hp | pct) | \(delta($bp; $hp)) |" ),
  "",
  "<details><summary>Line coverage per file</summary>",
  "",
  "| File | Base | This PR | Change |",
  "|---|---|---|---|",
  ( files($b) as $bf | files($h) as $hf
    | ($bf + $hf | keys[]) as $f
    | "| `\($f)` | \($bf[$f] | pct) | \($hf[$f] | pct) | \(delta($bf[$f]; $hf[$f])) |" ),
  "",
  "</details>",
  "",
  "_Coverage is reported, not enforced. Test code is included in the numbers._"
'
