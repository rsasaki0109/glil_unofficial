# GNSS Factors

This fork ships a minimal, loose-coupled GNSS prior injector so that a
pre-projected position stream (ENU or any local Cartesian frame) can be
attached to the global mapping factor graph without tight receiver-level
integration.

## Scope

This is intentionally the simplest usable GNSS path:

- **Loose coupling only.** One `gtsam::GPSFactor` prior per submap pose, with
  per-axis or isotropic sigma taken from the CSV row. No velocity coupling,
  no lever-arm, no antenna offset, no bias modeling.
- **Local nav frame only.** The CSV must already be in a local Cartesian
  frame (ENU, UTM-local, site frame). This fork does not perform
  latitude/longitude projection; do that at the writer side.
- **One prior per submap.** A GNSS observation is matched to the nearest
  submap origin stamp within `time_tolerance`; `consume_once=true`
  prevents the same GNSS observation from constraining two submaps.

The module is disabled by default, so existing GLIL runs do not change
unless it is explicitly loaded via the `extension_modules` list in
`config_ros.json`.

## CSV Schema

Two row shapes are accepted. Header row, blank lines, and `#` comment lines
are skipped when `skip_header=true` (default) and `skip_invalid_rows=true`.

Per-axis sigma:

```csv
stamp,x,y,z,sigma_x,sigma_y,sigma_z
12.3,100.0,50.0,2.0,0.05,0.05,0.10
12.4,100.1,50.0,2.0,0.05,0.05,0.10
```

Isotropic sigma:

```csv
stamp,x,y,z,sigma
12.3,100.0,50.0,2.0,0.10
```

| column | meaning |
|---|---|
| `stamp` | sensor timestamp in seconds, must be ascending |
| `x`, `y`, `z` | position in the local navigation frame, meters |
| `sigma_*` or `sigma` | standard deviation in meters (positive, finite) |

Rows with a zero/negative/non-finite sigma, non-finite stamp, or non-finite
position are rejected by `GNSSObservation::valid()`.

## Config

`config_gnss.json` lives in the same directory as the rest of the GLIL
config and is loaded by the `gnss_prior_injector` extension module:

```json
{
  "gnss_prior_injector": {
    "enabled": true,
    "csv_path": "gnss.csv",
    "time_tolerance": 0.05,
    "consume_once": true,
    "skip_header": true,
    "skip_invalid_rows": true,
    "fail_on_load_error": false,
    "min_sigma": 0.001
  }
}
```

| key | default | meaning |
|---|---|---|
| `enabled` | `false` | master switch; when `false` the module only logs and returns |
| `csv_path` | `""` | CSV path, resolved relative to the config root |
| `time_tolerance` | `0.05` | max `\|obs.stamp - submap.stamp\|` in seconds |
| `consume_once` | `true` | if `true`, each GNSS observation can attach to only one submap |
| `skip_header` | `true` | skip the CSV header row automatically |
| `skip_invalid_rows` | `false` | keep loading after parse errors |
| `fail_on_load_error` | `false` | disable injector if any parse errors occurred |
| `min_sigma` | `0.001` | floor applied to each axis of the noise model |

## Wiring Into A Run

1. Place `gnss.csv` and `config_gnss.json` next to the other `config_*.json`
   files in your config root.
2. Edit `config_ros.json` and add the shared library to the extension list:

   ```json
   "extension_modules": ["libgnss_prior_injector.so"]
   ```

3. Start GLIL as usual. The injector logs `GNSS prior injector loaded N
   observations` on startup and `GNSS prior injection poses=... inserted=...`
   every time a submap update binds new priors.

The bundled fixture `config/sample_gnss_observations.csv` is used by the
`test_gnss_observation` CTest target to exercise the CSV parser and factor
construction end-to-end.

## What This Is Not

- Not a replacement for tight GNSS-IMU coupling. Position-only priors help
  constrain global drift but cannot recover heading from a single receiver.
- Not a geodetic-to-local projector. Latitude/longitude-to-meters conversion
  is out of scope and must be done before writing the CSV.
- Not evaluated against a published dataset here. The module is functional
  and tested at the CSV + factor level; real-run accuracy numbers are a
  follow-up.
