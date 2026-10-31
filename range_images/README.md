# lidar_sim

Virtual RoboSense RS-Helios 32. Ray-traces a scene and writes the MSOP/DIFOP
UDP stream the real sensor would send, as a `.pcap` that
`rslidar_sdk_node` replays exactly like a capture from the real unit.

Ported from ev-sim's LiDAR pipeline (`LiDAR3d.js` / `Lidar3dShader.js` ray
casting, `LidarProducts.js` noise, `PcapEncoder.js` framing). The packet
encoding is the inverse of `RsHeliosDecoder`, and the channel calibration is
our unit's, read from the DIFOP packets in `test_cloud.pcap`.

## Running

```sh
cd lidar_sim
cargo run --release                                  # writes out/sim.pcap (5 s, 10 Hz)
cargo run --release -- --visibility 50 --rain 0.05   # fog + rain
cargo run --release -- --help
```

Then, from the repo root, decode and view it with the real pipeline:

```sh
cargo run --release --bin zenoh_wrapper -- lidar_sim/sim_config.yaml
cargo run --release --bin zenoh_viz
```

`sim_config.yaml` is `src/rslidar/config/config.yaml` with `pcap_path`
pointing at `lidar_sim/out/sim.pcap` and the costmap `sensor_z` set to the
simulated 1.5 m mount height.

| Flag | Default | Meaning |
|---|---|---|
| `--revolutions` | 50 | scan length |
| `--rpm` | 600 | 300 / 600 / 1200 (5 / 10 / 20 Hz) |
| `--height` | 1.5 | mount height above ground (m) |
| `--sigma`, `--bias` | 0.02, 0 | range noise floor and bias (m) |
| `--sigma-threshold` | 0.05 | extra range noise at the detection threshold, ∝ 1/SNR (m) |
| `--divergence` | 3 | full-angle beam divergence at 1/e² (mrad); 0 = ideal ray |
| `--footprint-rings` | 2 | rings of sub-rays sampling the footprint (1 + 6 + 12 rays) |
| `--pulse-width` | 5 | laser pulse width (ns); sets the 0.75 m echo resolution |
| `--dropout` | 0 | per-return dropout probability |
| `--range-10pct` | 110 | detection range of a 10% target in clear air (m) |
| `--visibility` | none | fog visibility (m) |
| `--rain` | 0 | raindrop returns per metre of beam path |
| `--seed` | 1 | noise RNG seed |
| `--scene` | parking | `parking`, or test scenes `plates` / `poles` |
| `--target-rho` | 0.1 | test-target reflectivity |
| `--target-angle` | 0 | plate incidence angle (deg) |
| `--target-radius` | 0.01 | pole radius (m) |

The scene (`scene::demo_scene`) is a parking lot: ground, a building, a wall,
parked and moving cars (boxes), pedestrians and poles (cylinders), a
retroreflective sign, and a 0.5 m calibration sphere.

The test scenes place 60 targets around the sensor, at sensor height and
with no ground, at ranges spaced geometrically from 2 to 160 m. Each target
sits in its own 6° azimuth sector. `plates` are large enough to fill the
beam footprint; `poles` are thin enough to be partial targets. Use them to
sweep one variable at a time against the analytic model.

## Report figures

```sh
python3 lidar_sim/analysis/plots.py             # all figures (~3 min first run, cached after)
python3 lidar_sim/analysis/plots.py fog rain    # just these
```

The script runs the simulator per experiment, with pcaps cached in
`out/analysis/`. It decodes them with the driver's equations and writes PDF
and PNG figures, plus `results.json`, to `documentation/figures/sim/`.
Experiments: `fov real rpm height reflectivity fog incidence footprint
noise rain motion`.

## Model

**Field of view.** 32 beams at fixed elevations φ_c (−15.98° to +9.99°,
non-uniform) spinning about z. Beam c in a block at azimuth θ_b fires at
t = t_b + τ_c (τ_c ∈ [0, 53.63] µs), at azimuth

θ_c = θ_b + Δθ · τ_c / 55.56 µs + δ_c

where Δθ is the per-block azimuth step (0.20° at 600 rpm) and δ_c the
channel's horizontal calibration offset. Its ray leaves the lens center,
offset R_x = 34.98 mm from the axis:

o_c = (R_x cos θ_c⁰, −R_x sin θ_c⁰, 0),
d_c = (cos φ_c cos θ_c, −cos φ_c sin θ_c, sin φ_c)

so the horizontal FOV is 360° in 1800 columns, and the vertical FOV is 26°.

**Geometry.** The range r is the nearest exact intersection of o + t·d with
the scene: plane, oriented-box slab test, vertical cylinder and cap, sphere.
Moving objects are evaluated at each beam's own firing time, so they show
rolling-shutter skew like the real sensor. The incidence angle comes from
the surface normal n, as cos α = |n · d|.

**Beam footprint.** Each pulse is a Gaussian beam with 1/e² half-angle
γ = divergence/2, so its footprint radius grows as w(r) = r·γ (spherical
spreading). The footprint is sampled with sub-rays on 1 + 6 + 12 rings out
to the 1/e² radius. Each sub-ray i carries the energy fraction
w_i ∝ A_i e^(−2ρ_i²/γ²), normalized so Σ w_i = 1, and is traced like the
central ray.

**Detection.** From the LiDAR range equation, a sub-ray hitting a
Lambertian patch with reflectivity ρ returns the power

P_i ∝ w_i ρ cos α · e^(−2σr) / r²

The receiver cannot separate contributions closer than c·τ_p/2 = 0.75 m.
Those merge into one echo, with P = Σ P_i at the power-weighted range.
This produces mixed pixels at object edges, and range walk on grazing
surfaces. A target that fills the footprint gets Σ w_i = 1 and the 1/r²
law. A target smaller than the footprint intercepts only part of the
energy, so its return falls faster (≈ 1/r³ for a pole, 1/r⁴ for a point).
The strongest echo with P ≥ P_min is reported, matching the unit's
strongest-return mode. P_min is set so that a ρ = 0.1
target at normal incidence is just detectable at `--range-10pct` in clear
air. The atmospheric extinction σ comes from visibility V by Koschmieder's
law, σ = 3.912 / V. Fog therefore shortens the detection range, and it does
so more for dark, oblique targets.

**Rain.** Raindrops along the beam form a Poisson process with density λ
(`--rain`). The first drop sits at r_d ~ Exp(λ). If it is in front of the
nearest surface, it adds a weak echo (ρ_d = 2·10⁻⁴) that is detectable only
for r_d ≲ 5 m. It is reported as a near-range, intensity-1 point when it is
the strongest echo, which happens mostly where the beam would otherwise
return nothing (sky) or only a weak return.

**Measurement.** Range comes from time of flight: τ = 2r/c is measured
with Gaussian jitter, and r̂ = c·τ̂/2. The equivalent range noise is

σ_r(P) = √(σ_0² + (σ_th · P_min/P)²)

That is a floor σ_0 plus leading-edge timing jitter ∝ 1/SNR, so weak, far,
dark or oblique returns are noisier. r̂ is quantized to the protocol's
2.5 mm. Intensity is range-compensated return power,
I = 100·P·r² = 100·ρ·cos α·e^(−2σr)·Σw_hit: 0–100 for diffuse surfaces,
and above 100 for retroreflectors (ρ > 1).

**Output.** 12 blocks × 32 channels per 1248-byte MSOP packet, with the
LiDAR clock in the header. There is a DIFOP packet (RPM, FOV, calibration)
once per second. Each packet is wrapped in Ethernet/IPv4/UDP to ports
6699/7788 in a classic pcap, with the record time stamped at the end of the
packet.

## Verification

Decoding the simulated pcap with the decoder's own equations puts points
back on the true geometry. With `--divergence 0 --sigma-threshold 0`, the
output is byte-identical to the original ideal-ray model. These numbers
are from revolution 1 with `--sigma 0`:

- **Open ground (ideal ray):** error is 0.16 mm RMS. That is the 2.5 mm
  range quantization projected onto the ground.
- **Calibration sphere:** surface error is 0.52 mm RMS with an ideal ray,
  and 1.01 mm with the 3 mrad footprint (mixed pixels at the limb).
- **Footprint vs. ideal ray, same beams:** the median range change is 0.
  Changes over 10 cm (edge mixed pixels) affect 0.04% of returns at
  5–10 m, 0.27% at 10–20 m and 1.7% at 20–40 m, growing with the footprint.
- **Fog, V = 50 m:** maximum range drops from 35.7 m to 23.3 m. The
  retroreflective sign at 10 m dims from intensity 244 to 50, matching
  e^(−2σr).

`zenoh_wrapper` decodes 57,600 points per frame at 10 Hz from it.
