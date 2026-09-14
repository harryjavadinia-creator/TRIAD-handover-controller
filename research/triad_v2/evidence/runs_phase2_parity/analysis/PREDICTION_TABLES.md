### Worst case over valid decision times while the object moves

| condition | source | peak accel (m/s²) | decision times | estimate speed error max / p95 (mm/s) | h_valid: max e_p ≤ 5 / 10 / 15 / 30 mm (s) | h_valid: p95 e_p ≤ 15 mm (s) | max e_p at h = 0 / 0.5 / 1.0 / 1.8 / 3.0 s (mm) | h_valid e_R ≤ 0.05 / 0.12 rad (s) |
|---|---|---:|---:|---|---|---:|---|---|
| speed 0.04 m/s, stop 0.85 s | SIMULATOR | 0.088 | 952 | 8.1 / 1.4 | 0.25 / 0.40 / 0.55 / 1.00 | 0.75 | 0.0 / 11.9 / 29.8 / 60.4 / 107.5 | 8.00 / 8.00 |
| speed 0.08 m/s, stop 0.85 s | SIMULATOR | 0.176 | 457 | 16.2 / 14.1 | 0.15 / 0.25 / 0.35 / 0.55 | 0.35 | 0.0 / 23.8 / 59.7 / 120.9 / 214.9 | 8.00 / 8.00 |
| speed 0.12 m/s, stop 0.85 s | SIMULATOR | 0.265 | 292 | 24.3 / 23.0 | 0.10 / 0.20 / 0.25 / 0.40 | 0.25 | 0.0 / 35.6 / 89.5 / 181.3 / 322.4 | 8.00 / 8.00 |
| speed 0.16 m/s, stop 0.85 s | SIMULATOR | 0.353 | 210 | 32.4 / 31.5 | 0.10 / 0.15 / 0.20 / 0.35 | 0.20 | 0.0 / 47.5 / 119.3 / 241.7 / 429.8 | 8.00 / 8.00 |
| speed 0.24 m/s, stop 0.85 s | SIMULATOR | 0.529 | 129 | 48.6 / 48.0 | 0.05 / 0.10 / 0.15 / 0.25 | 0.15 | 0.0 / 71.3 / 179.0 / 362.6 / 644.7 | 8.00 / 8.00 |
| speed 0.08 m/s, stop 0.425 s | SIMULATOR | 0.353 | 444 | 27.6 / 8.7 | 0.10 / 0.20 / 0.25 / 0.45 | 0.30 | 0.0 / 31.1 / 69.3 / 132.1 / 227.2 | 8.00 / 8.00 |
| speed 0.08 m/s, stop 1.7 s | SIMULATOR | 0.088 | 483 | 8.6 / 8.3 | 0.25 / 0.35 / 0.50 / 0.75 | 0.50 | 0.0 / 14.4 / 43.6 / 100.5 / 191.7 | 8.00 / 8.00 |
| speed 0.08 m/s, stop 3.0 s | SIMULATOR | 0.050 | 524 | 5.0 / 4.9 | 0.35 / 0.50 / 0.65 / 1.00 | 0.70 | 0.0 / 8.6 / 28.1 / 75.4 / 160.4 | 8.00 / 8.00 |
| speed 0.08 m/s, yaw 0.30 rad/s, stop 0.85 s | SIMULATOR | 0.176 | 457 | 16.2 / 14.1 | 0.15 / 0.25 / 0.35 / 0.55 | 0.35 | 0.0 / 23.8 / 59.7 / 120.9 / 214.9 | 0.30 / 0.60 |
| speed 0.08, delay 0.1 s, compensated | SIMULATOR | 0.176 | 457 | 32.0 / 27.9 | 0.05 / 0.15 / 0.25 / 0.45 | 0.25 | 2.4 / 30.6 / 67.2 / 128.6 / 222.8 | 8.00 / 8.00 |
| speed 0.08, delay 0.22 s, compensated | SIMULATOR | 0.176 | 457 | 48.9 / 41.4 | 0.00 / 0.05 / 0.10 / 0.35 | 0.15 | 7.2 / 39.1 / 76.2 / 138.0 / 232.3 | 8.00 / 8.00 |
| speed 0.08, delay 0.22 s, uncompensated | SIMULATOR | 0.176 | 457 | 48.9 / 41.4 | 0.00 / 0.00 / 0.00 / 0.55 | 0.00 | 17.6 / 23.8 / 59.7 / 120.9 / 214.9 | 8.00 / 8.00 |
| speed 0.08, per-tick noise sigma_p=0.2 mm sigma_R=0.0 rad (1 kHz) | OFFLINE_ONLY | 0.176 | 457 | 93.0 / 85.0 | 0.05 / 0.10 / 0.15 / 0.30 | 0.15 | 0.8 / 46.6 / 93.1 / 167.4 / 279.0 | 8.00 / 8.00 |
| speed 0.08, per-tick noise sigma_p=1.0 mm sigma_R=0.0 rad (1 kHz) | OFFLINE_ONLY | 0.176 | 457 | 85.7 / 85.7 | 0.00 / 0.05 / 0.10 / 0.30 | 0.15 | 3.9 / 45.6 / 87.3 / 155.6 / 252.3 | 8.00 / 8.00 |
| speed 0.08, per-tick noise sigma_p=2.0 mm sigma_R=0.01 rad (1 kHz) | OFFLINE_ONLY | 0.176 | 457 | 80.0 / 80.0 | 0.00 / 0.00 / 0.10 / 0.30 | 0.10 | 8.4 / 45.5 / 85.4 / 149.3 / 245.1 | 8.00 / 8.00 |
| speed 0.08, noise-free sample-and-hold at 30 Hz | OFFLINE_ONLY | 0.176 | 457 | 80.0 / 80.0 | 0.00 / 0.05 / 0.15 / 0.30 | 0.15 | 2.6 / 42.6 / 82.6 / 146.6 / 242.6 | 8.00 / 8.00 |
| speed 0.08, noise-free sample-and-hold at 100 Hz | OFFLINE_ONLY | 0.176 | 457 | 80.0 / 80.0 | 0.05 / 0.10 / 0.15 / 0.35 | 0.15 | 0.0 / 40.0 / 80.0 / 144.0 / 240.0 | 8.00 / 8.00 |
| speed 0.08, quintic start 0.5 s, stop 0.85 s | OFFLINE_ONLY | 0.300 | 507 | 16.2 / 13.6 | 0.15 / 0.25 / 0.35 / 0.55 | 0.40 | 0.0 / 23.8 / 59.7 / 120.9 / 214.9 | 8.00 / 8.00 |
| speed 0.08, heading turn 0.25 rad/s after 1 s (no stop) | OFFLINE_ONLY | 0.020 | 531 | 2.0 / 2.0 | 0.60 / 0.90 / 1.10 / 1.60 | 1.10 | 0.0 / 3.5 / 12.0 / 35.8 / 94.4 | 8.00 / 8.00 |
| speed 0.08, heading turn 0.5 rad/s after 1 s (no stop) | OFFLINE_ONLY | 0.040 | 531 | 4.0 / 4.0 | 0.40 / 0.60 / 0.75 / 1.10 | 0.75 | 0.0 / 7.0 / 23.8 / 70.2 / 179.4 | 8.00 / 8.00 |

### Error when the whole horizon stays inside the constant-velocity segment (estimator + latency error only)

| condition | max e_p at h = 0 / 0.5 / 1.0 / 1.8 / 3.0 s (mm; '—' = no such decision time) |
|---|---|
| speed 0.04 m/s, stop 0.85 s | 0.00 / 0.02 / 0.04 / 0.07 / 0.11 |
| speed 0.08 m/s, stop 0.85 s | 0.00 / 0.04 / 0.08 / 0.14 / 0.23 |
| speed 0.12 m/s, stop 0.85 s | 0.00 / 0.06 / 0.11 / 0.20 / — |
| speed 0.16 m/s, stop 0.85 s | 0.00 / 0.08 / 0.15 / — / — |
| speed 0.24 m/s, stop 0.85 s | 0.00 / 0.11 / — / — / — |
| speed 0.08 m/s, stop 0.425 s | 0.00 / 0.04 / 0.08 / 0.14 / 0.23 |
| speed 0.08 m/s, stop 1.7 s | 0.00 / 0.04 / 0.08 / 0.14 / 0.23 |
| speed 0.08 m/s, stop 3.0 s | 0.00 / 0.04 / 0.08 / 0.14 / — |
| speed 0.08 m/s, yaw 0.30 rad/s, stop 0.85 s | 0.00 / 0.04 / 0.08 / 0.14 / 0.23 |
| speed 0.08, delay 0.1 s, compensated | 0.02 / 0.12 / 0.22 / 0.39 / 0.63 |
| speed 0.08, delay 0.22 s, compensated | 0.15 / 0.49 / 0.82 / 1.36 / 2.17 |
| speed 0.08, delay 0.22 s, uncompensated | 17.60 / 17.94 / 18.27 / 18.81 / 19.62 |
| speed 0.08, per-tick noise sigma_p=0.2 mm sigma_R=0.0 rad (1 kHz) | 0.79 / 46.59 / 93.07 / 167.43 / 278.98 |
| speed 0.08, per-tick noise sigma_p=1.0 mm sigma_R=0.0 rad (1 kHz) | 3.95 / 45.62 / 87.32 / 155.61 / 251.07 |
| speed 0.08, per-tick noise sigma_p=2.0 mm sigma_R=0.01 rad (1 kHz) | 7.54 / 45.47 / 85.40 / 149.10 / 245.09 |
| speed 0.08, noise-free sample-and-hold at 30 Hz | 2.56 / 42.56 / 82.56 / 146.56 / 242.56 |
| speed 0.08, noise-free sample-and-hold at 100 Hz | 0.00 / 40.00 / 80.00 / 144.00 / 240.00 |
| speed 0.08, quintic start 0.5 s, stop 0.85 s | 0.00 / 0.69 / 1.39 / 2.50 / 4.16 |
| speed 0.08, heading turn 0.25 rad/s after 1 s (no stop) | 0.00 / — / — / — / — |
| speed 0.08, heading turn 0.5 rad/s after 1 s (no stop) | 0.00 / — / — / — / — |

### Worst e_p (mm) by time from the decision to the onset of the unmodelled acceleration (bins of 0.5 s; negative = decision after onset)


h = 1.0 s

| condition | -1.0 | -0.5 | +0.0 | +0.5 | +1.0 | +1.5 | +2.0 | +2.5 | +3.0 | +3.5 | +4.0 | +4.5 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| speed 0.04 m/s, stop 0.85 s | 20 | 30 | 23 | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| speed 0.08 m/s, stop 0.85 s | 40 | 60 | 46 | 9 | 0 | 0 | 0 | 0 | 0 | 0 |  |  |
| speed 0.12 m/s, stop 0.85 s | 60 | 89 | 68 | 13 | 0 | 0 | 0 |  |  |  |  |  |
| speed 0.16 m/s, stop 0.85 s | 79 | 119 | 91 | 17 | 0 |  |  |  |  |  |  |  |
| speed 0.24 m/s, stop 0.85 s | 118 | 179 | 138 | 26 |  |  |  |  |  |  |  |  |
| speed 0.08 m/s, stop 0.425 s | 6 | 69 | 62 | 22 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |  |
| speed 0.08 m/s, stop 1.7 s | 44 | 43 | 18 | 2 | 0 | 0 | 0 | 0 | 0 |  |  |  |
| speed 0.08 m/s, stop 3.0 s | 27 | 17 | 5 | 0 | 0 | 0 | 0 | 0 |  |  |  |  |
| speed 0.08 m/s, yaw 0.30 rad/s, stop 0.85 s | 40 | 60 | 46 | 9 | 0 | 0 | 0 | 0 | 0 | 0 |  |  |
| speed 0.08, delay 0.1 s, compensated | 57 | 67 | 46 | 9 | 0 | 0 | 0 | 0 | 0 | 0 |  |  |
| speed 0.08, delay 0.22 s, compensated | 75 | 76 | 46 | 9 | 0 | 0 | 0 | 0 | 0 | 1 |  |  |
| speed 0.08, delay 0.22 s, uncompensated | 59 | 60 | 28 | 18 | 18 | 18 | 18 | 18 | 18 | 18 |  |  |
| speed 0.08, per-tick noise sigma_p=0.2 mm sigma_R=0.0 rad (1 kHz) | 22 | 26 | 64 | 81 | 82 | 77 | 62 | 69 | 89 | 93 |  |  |
| speed 0.08, per-tick noise sigma_p=1.0 mm sigma_R=0.0 rad (1 kHz) | 10 | 40 | 78 | 85 | 87 | 86 | 87 | 87 | 83 | 81 |  |  |
| speed 0.08, per-tick noise sigma_p=2.0 mm sigma_R=0.01 rad (1 kHz) | 7 | 35 | 70 | 84 | 85 | 85 | 84 | 84 | 85 | 83 |  |  |
| speed 0.08, noise-free sample-and-hold at 30 Hz | 3 | 36 | 72 | 83 | 83 | 83 | 83 | 83 | 83 | 83 |  |  |
| speed 0.08, noise-free sample-and-hold at 100 Hz | 13 | 34 | 71 | 80 | 80 | 80 | 80 | 80 | 80 | 80 |  |  |
| speed 0.08, quintic start 0.5 s, stop 0.85 s | 40 | 60 | 46 | 9 | 0 | 0 | 0 | 0 | 0 | 0 | 1 |  |
| speed 0.08, heading turn 0.25 rad/s after 1 s (no stop) | 12 | 12 | 10 |  |  |  |  |  |  |  |  |  |
| speed 0.08, heading turn 0.5 rad/s after 1 s (no stop) | 24 | 24 | 20 |  |  |  |  |  |  |  |  |  |

h = 1.8 s

| condition | -1.0 | -0.5 | +0.0 | +0.5 | +1.0 | +1.5 | +2.0 | +2.5 | +3.0 | +3.5 | +4.0 | +4.5 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| speed 0.04 m/s, stop 0.85 s | 37 | 60 | 55 | 35 | 15 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |
| speed 0.08 m/s, stop 0.85 s | 74 | 121 | 110 | 70 | 30 | 2 | 0 | 0 | 0 | 0 |  |  |
| speed 0.12 m/s, stop 0.85 s | 112 | 181 | 164 | 104 | 44 | 2 | 0 |  |  |  |  |  |
| speed 0.16 m/s, stop 0.85 s | 147 | 242 | 219 | 139 | 59 |  |  |  |  |  |  |  |
| speed 0.24 m/s, stop 0.85 s | 218 | 363 | 330 | 209 |  |  |  |  |  |  |  |  |
| speed 0.08 m/s, stop 0.425 s | 11 | 132 | 126 | 86 | 46 | 7 | 0 | 0 | 0 | 0 | 0 |  |
| speed 0.08 m/s, stop 1.7 s | 100 | 101 | 76 | 37 | 9 | 0 | 0 | 0 | 0 |  |  |  |
| speed 0.08 m/s, stop 3.0 s | 75 | 62 | 33 | 12 | 2 | 0 | 0 | 0 |  |  |  |  |
| speed 0.08 m/s, yaw 0.30 rad/s, stop 0.85 s | 74 | 121 | 110 | 70 | 30 | 2 | 0 | 0 | 0 | 0 |  |  |
| speed 0.08, delay 0.1 s, compensated | 104 | 129 | 110 | 70 | 30 | 2 | 0 | 0 | 0 | 0 |  |  |
| speed 0.08, delay 0.22 s, compensated | 132 | 138 | 110 | 70 | 30 | 2 | 0 | 0 | 0 | 1 |  |  |
| speed 0.08, delay 0.22 s, uncompensated | 117 | 121 | 92 | 52 | 16 | 18 | 18 | 18 | 18 | 19 |  |  |
| speed 0.08, per-tick noise sigma_p=0.2 mm sigma_R=0.0 rad (1 kHz) | 40 | 33 | 69 | 108 | 144 | 139 | 111 | 125 | 160 | 167 |  |  |
| speed 0.08, per-tick noise sigma_p=1.0 mm sigma_R=0.0 rad (1 kHz) | 16 | 45 | 85 | 120 | 149 | 153 | 156 | 155 | 150 | 145 |  |  |
| speed 0.08, per-tick noise sigma_p=2.0 mm sigma_R=0.01 rad (1 kHz) | 7 | 35 | 72 | 115 | 143 | 149 | 148 | 148 | 149 | 147 |  |  |
| speed 0.08, noise-free sample-and-hold at 30 Hz | 3 | 36 | 74 | 114 | 144 | 147 | 147 | 147 | 147 | 147 |  |  |
| speed 0.08, noise-free sample-and-hold at 100 Hz | 23 | 34 | 74 | 114 | 142 | 144 | 144 | 144 | 144 | 144 |  |  |
| speed 0.08, quintic start 0.5 s, stop 0.85 s | 74 | 121 | 110 | 70 | 30 | 2 | 0 | 0 | 0 | 0 | 2 |  |
| speed 0.08, heading turn 0.25 rad/s after 1 s (no stop) | 36 | 36 | 32 |  |  |  |  |  |  |  |  |  |
| speed 0.08, heading turn 0.5 rad/s after 1 s (no stop) | 70 | 70 | 63 |  |  |  |  |  |  |  |  |  |
