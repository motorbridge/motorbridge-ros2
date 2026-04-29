# abi directory

This directory is for generated ABI artifacts only.

## What is generated here

`build.rs` builds `motorbridge` package `motor_abi` and copies the platform artifact into this folder:

- Windows: `motor_abi.dll`
- Linux: `libmotor_abi.so`
- macOS: `libmotor_abi.dylib`

## Git policy

Generated ABI binaries are ignored by git (`.gitignore`), so this folder should normally contain only this README in source control.

## Runtime override

If needed, set `MOTORBRIDGE_ABI_PATH` to use an external ABI binary path.
