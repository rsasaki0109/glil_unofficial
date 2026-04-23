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

## Reproduction Reports

If you try another dataset, open a
[reproduction report](https://github.com/rsasaki0109/glil_unofficial/issues/new?template=reproduction_report.md)
with the dataset name, config, RMSE or smoke markers, playback speed, and
trajectory hash. Reports that include enough metadata can be promoted into this
scoreboard.
