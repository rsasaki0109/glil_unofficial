# Reproduction Scoreboard

This page is the short, shareable evidence view for the GLIL CPU reproduction
fork. It is designed for readers deciding whether the fork is worth trying or
starring.

![Reproduction scorecard](assets/reproduction-scorecard.svg)

## Headline

- MegaParticles APE checks: 3/3 PASS for Track B and Track C.
- Deterministic hard recipe: `outdoor_hard_01a` is 5/5 byte-identical with RMSE `0.906313`.
- Official Ouster sample: completes cleanly with `1122` pose rows, fallback `0`, bitset abort `0`.
- The Ouster smoke result is WARN only because there is no bundled ground-truth APE file for that sample.

## Results

| dataset | kind | status | RMSE | upstream GLIM | playback mean | Track B | Track C | recommended config |
|---|---|---|---:|---:|---:|---|---|---|
| `indoor_easy_01` | APE | PASS | `1.019250` | `3.383012` | `1.000x` | PASS | PASS | `config_fair_glil_true_sample_t128_indoor_d4k_k1_rw_csp15_ct64_lag4` |
| `outdoor_hard_01a` | APE | PASS | `0.906313` | `4.321651` | `1.000x` | PASS | PASS | `config_fair_glil_true_sample_t128_hard_csp15_ct64_lag4_ffb100_skip16` |
| `outdoor_kidnap_a` | APE | PASS | `20.349845` | `21.701012` | `1.000x` | PASS | PASS | `config_fair_glil_true_sample_t128_k1` |
| `os1_128_01_downsampled` | smoke | WARN | NA | NA | `0.201x` | NA | NA | `config_official_os1_128_01_downsampled_acc1` |

Track B means the GLIL RMSE is within upstream GLIM RMSE + 20%. Track C means
mean playback is at least `0.95x`.

## What To Try First

1. Start with `outdoor_hard_01a` and `config_fair_glil_true_sample_t128_hard_csp15_ct64_lag4_ffb100_skip16` if you want the strongest determinism signal.
2. Use `indoor_easy_01` when you want a quick accuracy sanity check with a large margin over the upstream GLIM baseline.
3. Use `os1_128_01_downsampled` when you want an official sample smoke check and IMU scaling guardrail.

## Shareable Summary

```text
GLIL CPU Reproduction Fork: reproducible LiDAR odometry configs for MegaParticles and Ouster smoke checks. 3/3 Track B+C PASS, outdoor_hard_01a RMSE 0.906313, and the hard recipe is 5/5 byte-identical.
```

## Perception Readiness

For perception-enabled runs, generate a compact readiness table and include it
next to RMSE, playback, and trajectory hash evidence:

```bash
glil_perception_report_summary \
  --report dataset_a=dataset_a/perception_report.csv \
  --report dataset_b=dataset_b/perception_report.csv \
  --format markdown \
  --output perception_summary.md
```

This keeps accepted factor counts, rejected-observation reasons, and timestamp
match rates visible before a run is promoted into the scoreboard.

## Reproduction Bundle

Use `glil_reproduction_bundle` to package the command, config pointer,
perception readiness, trajectory metrics, benchmark CSV, and artifact list for a
run before opening a reproduction issue:

```bash
glil_reproduction_bundle \
  --run-dir dataset_a \
  --command-log dataset_a/command.txt \
  --ape dataset_a/evo_ape.txt \
  --perception-report dataset_a/perception_report.csv \
  --benchmark-csv dataset_a/coreset_benchmark.csv \
  --format markdown \
  --output dataset_a/reproduction_bundle.md
```

The bundle keeps paths display-safe by preferring run-directory or repository
relative paths, and it marks missing supplied artifacts as warnings instead of
silently ignoring them. Use `--format json` when a script needs to ingest the
same evidence.

### Example: outdoor_hard_01a reproduction

A bundle generated from the `outdoor_hard_01a` reproduction run
(`results/latest_glim/outdoor_hard_01a_reproduce_20260421/`) looks like this in
Markdown:

```markdown
# GLIL outdoor_hard_01a reproduction (2026-04-21)

| field | value |
|---|---|
| status | PASS |
| git commit | `a97ec89bd24a98bfd1a1fe494a7b21d1a89398b9` |
| run directory | `results/latest_glim/outdoor_hard_01a_reproduce_20260421` |

## Artifacts

| kind | label | path | exists | bytes |
|---|---|---|---:|---:|
| command | reproduce_glil | `scripts/reproduce_glil.sh` | yes | 6168 |
| metric | evo_ape | `evo_ape.txt` | yes | 498 |
| artifact | git_ref | `git_ref.txt` | yes | 239 |
| artifact | contract_playback | `contract_playback.txt` | yes | 28 |
| artifact | summary | `reproduce_glil_summary_20260421.md` | yes | 925 |
| artifact | reproduce_manifest | `reproduce_manifest.json` | yes | 2933 |

## Trajectory Metrics

| label | extracted metrics |
|---|---|
| evo_ape | max=2.692416, mean=0.796697, median=0.751121, min=0.107598, rmse=0.906313, sse=2347.571431, std=0.432062 |

## Notes

- Track B + Track C PASS (RMSE 0.906313 m vs upstream GLIM 4.321651 m; playback 1.0000x).
- Trajectory byte-identical across 5/5 repeats (BZ5 cached immutable).
- Dataset: MegaParticles outdoor_hard_01a; config `config_fair_glil_true_sample_t128_hard_csp15_ct64_lag4_ffb100_skip16`.
```

The command that produced it:

```bash
glil_reproduction_bundle \
  --repo-root . \
  --run-dir results/latest_glim/outdoor_hard_01a_reproduce_20260421 \
  --ape results/latest_glim/outdoor_hard_01a_reproduce_20260421/evo_ape.txt \
  --command-log scripts/reproduce_glil.sh \
  --git-commit a97ec89bd24a98bfd1a1fe494a7b21d1a89398b9 \
  --artifact git_ref=results/latest_glim/outdoor_hard_01a_reproduce_20260421/git_ref.txt \
  --artifact contract_playback=results/latest_glim/outdoor_hard_01a_reproduce_20260421/contract_playback.txt \
  --artifact summary=results/latest_glim/reproduce_glil_summary_20260421.md \
  --artifact reproduce_manifest=reproduce_manifest.json \
  --title "GLIL outdoor_hard_01a reproduction (2026-04-21)" \
  --note "Track B + Track C PASS (RMSE 0.906313 m vs upstream GLIM 4.321651 m; playback 1.0000x)" \
  --note "Trajectory byte-identical across 5/5 repeats (BZ5 cached immutable)" \
  --format markdown \
  --output results/latest_glim/outdoor_hard_01a_reproduce_20260421/reproduction_bundle.md
```

This is the bundle shape expected when attaching evidence to a reproduction
issue: RMSE and playback evidence is front-loaded, artifact presence is
explicit, and every displayed path is repository- or run-directory relative.

The same tool was used to produce bundles for all three MegaParticles
reproduction runs:

| dataset | run directory | RMSE | Track B | Track C |
|---|---|---:|---|---|
| `indoor_easy_01` | `results/latest_glim/indoor_easy_01_quiet_trackc_retry4_20260421` | `1.019250` | PASS | PASS |
| `outdoor_hard_01a` | `results/latest_glim/outdoor_hard_01a_reproduce_20260421` | `0.906313` | PASS | PASS |
| `outdoor_kidnap_a` | `results/latest_glim/outdoor_kidnap_a_reproduce_20260421` | `20.349845` | PASS | PASS |

Each run directory contains a `reproduction_bundle.md` generated from the
command above with the per-dataset `--run-dir`, `--title`, and `--note` values
adjusted. Attach that bundle directly when filing a reproduction report for
that dataset.

## Reproduction Reports

If you try another dataset, open a
[reproduction report](https://github.com/rsasaki0109/glil_unofficial/issues/new?template=reproduction_report.md)
with the dataset name, config, RMSE or smoke markers, playback speed, and
trajectory hash. Reports that include enough metadata can be promoted into this
scoreboard.
