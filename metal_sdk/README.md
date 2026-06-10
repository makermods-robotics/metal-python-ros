# Metal Arm Python SDK

This package exposes the MakerMods Metal arm SDK to Python through a `pybind11` wrapper.

The package name and wrapper symbols remain `metal_sdk` and `MetalSDKInterface` for compatibility with the bundled native SDK binary.

## Install

```bash
pip install .
```

## Inspect

```bash
pip show metal_sdk
```

## Uninstall

```bash
pip uninstall metal_sdk
```

## SDK Interface

See `metal_sdk_interface.cpp` and `metal_sdk/metal_sdk_interface.h`.

The implementation is provided by the prebuilt native shared library in `metal_sdk/metal_sdk/lib/x64/`.

## Examples

Gravity-compensation mode, typically used as a leader arm for teaching:

```bash
cd metal_sdk/
python3 example/one_master.py
```

Real-time joint-position mode, typically used as a follower arm:

```bash
cd metal_sdk/
python3 example/one_slave.py
```

Non-real-time joint-position mode, recommended for normal scripted control and model inference:

```bash
cd metal_sdk/
python3 example/single_arm_control.py
```
