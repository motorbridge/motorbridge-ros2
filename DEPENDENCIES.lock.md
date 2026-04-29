# Dependency Lock Baseline

Last updated: 2026-04-29

## Repositories and pinned commits

- `motorbridge-ros2`
  - path: `.`
  - note: this repository head is the release baseline commit that includes this file.

- `third_party/motorbridge`
  - source: `https://github.com/tianrking/motorbridge.git`
  - commit: `a9667da323ae94fef9ee4390d23d5ed74cb1faa3`

- `third_party/RustDDS`
  - source: `https://github.com/Atostek/RustDDS.git`
  - commit: `20f787c813367a2f4358c7c35c9096fc94a2595f`

- `third_party/zenoh`
  - source: `https://github.com/eclipse-zenoh/zenoh.git`
  - commit: `7792ebbb2fb0c8311e51419994171cf91fb619d7`

## Cargo dependency style used by motorbridge-ros2

- path dependency: `third_party/motorbridge/motor_core`
- path dependency: `third_party/motorbridge/motor_vendors/damiao`
- path dependency: `third_party/RustDDS`

## Reproducibility notes

- Preferred: clone with submodules:
  - `git clone --recurse-submodules <repo-url>`
- Or run:
  - `git submodule update --init --recursive`
- Custom local source mode is supported via:
  - `scripts/bootstrap.ps1`
- Keep this file updated whenever submodule commit pointers change.
- Commit `Cargo.lock` together with dependency bumps.
