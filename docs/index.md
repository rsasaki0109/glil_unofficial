# Home

## Introduction

![GLIM](assets/logo2.png "GLIM Logo")

**GLIM** is a versatile and extensible range-based 3D mapping framework.

- ***Accuracy:*** GLIM is based on direct multi-scan registration error minimization on factor graphs that enables to accurately retain the consistency of mapping results. GPU acceleration is supported to maximize the mapping speed and quality.
- ***Easy-to-use:*** GLIM offers an interactive map correction interface that enables the user to manually correct mapping failures and easily refine mapping results.
- ***Versatility:*** As we eliminated sensor-specific processes, GLIM can be applied to any kind of range sensors including:
    - Spinning-type LiDAR (e.g., Velodyne HDL32e)
    - Non-repetitive scan LiDAR (e.g., Livox Avia)
    - Solid-state LiDAR (e.g., Intel Realsense L515)
    - RGB-D camera (e.g., Microsoft Azure Kinect)
- ***Extensibility:*** GLIM provides the global callback slot mechanism that allows to access the internal states of the mapping process and insert additional constraints to the factor graph. We also release [glim_ext](https://github.com/koide3/glim_ext) that offers example implementations of several extension functions (e.g., explicit loop detection, LiDAR-Visual-Inertial odometry estimation).

Tested on Ubuntu 22.04 / 24.04 with CUDA 12.2, and NVIDIA Jetson Orin.

## GLIL CPU Reproduction Fork

This fork adds CPU-focused reproduction configs and validation notes for local
LiDAR odometry experiments. The 2026-04 manifest-verified bundle records
3/3 MegaParticles APE PASS plus a clean official Ouster sample smoke check.

![Reproduction scorecard](assets/reproduction-scorecard.svg)

| dataset | kind | status | RMSE | playback | note |
|---|---|---|---:|---:|---|
| `indoor_easy_01` | APE | PASS | `1.019250` | `1.000x` | Track B+C PASS |
| `outdoor_hard_01a` | APE | PASS | `0.906313` | `1.000x` | 5/5 byte-identical hard recipe |
| `outdoor_kidnap_a` | APE | PASS | `20.349845` | `1.000x` | Track B+C PASS |
| `os1_128_01_downsampled` | smoke | WARN | NA | `0.201x` | clean Ouster smoke, no GT APE |

Track B is upstream GLIM RMSE + 20%. Track C is playback mean `>= 0.95x`.
The Ouster sample is a completion/stability smoke check because this workspace
does not include a matching ground-truth APE file. See the
[Reproduction scoreboard](reproduction.md) for the shareable summary and
recommended configs.

[![Build](https://github.com/rsasaki0109/glil_unofficial/actions/workflows/build.yml/badge.svg)](https://github.com/rsasaki0109/glil_unofficial/actions/workflows/build.yml)
[![GitHub stars](https://img.shields.io/github/stars/rsasaki0109/glil_unofficial?style=social)](https://github.com/rsasaki0109/glil_unofficial/stargazers)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/rsasaki0109/glil_unofficial/blob/dev/LICENSE)

## Video

### Robustness test
<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/Kk-K2rCXt-U" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

### Mapping with various range sensors

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/_fwK4awbW18?si=R5m5502i7sKTbopg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

### Outdoor driving test with Livox MID360

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/CIfRqeV0irE?si=WT-knUxMuGWjYcxQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

See more in [Extension modules](extensions.md) and [Demo](demo.md) pages.

## Contact

Kenji Koide [:material-home:](https://staff.aist.go.jp/k.koide/) [:material-mail:](mailto:k.koide@aist.go.jp) [:material-twitter:](https://twitter.com/k_koide3)  
National Institute of Advanced Industrial Science and Technology (AIST), Japan