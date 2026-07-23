# Building the Metal SDK from Source

The SDK has three parts, all in this directory:

- `native/` — the C++ SDK (CAN manager, motor drivers, kinodynamics, trajectory).
  A colcon/ament package that builds `libmetal_sdk_<arch>.so`.
- `binding/` — the pybind11 wrapper, pip-installable, links against the native library.
- `metal_sdk/` — the Python package shell; the built native library is copied into
  `metal_sdk/lib/<arch>/` (x64 or arm64, auto-detected).

## Dependencies

```bash
sudo apt install -y libgoogle-glog-dev libnlopt-cxx-dev \
  ros-humble-kdl-parser liborocos-kdl-dev libeigen3-dev \
  python3-pybind11 pybind11-dev
```

## One-shot build

```bash
bash build_metal_sdk.sh
```

This builds the native library with colcon, copies it into `metal_sdk/lib/<arch>/`,
and pip-installs the binding for the system Python 3.10 (`/usr/bin/python3`, the
interpreter ROS 2 Humble nodes run under).

## Manual steps (what the script does)

```bash
cd metal_sdk                                            # this directory
colcon build --packages-select metal_sdk --base-paths native
cp install/metal_sdk/x64/libmetal_sdk_x64.so metal_sdk/lib/x64/   # or arm64
/usr/bin/python3 -m pip install --user --no-build-isolation ./binding
```

Verify:

```bash
python3 -c "from metal_sdk import MetalSDKInterface, ControlMode; print('Success!')"
```

## Hardware test suite

`test/` contains a numbered bring-up suite (`t0`–`t8`) with escalating risk levels —
from CAN-up and read-only checks to gravity-compensation and small motions. See
`test/README.md`; run them in order on new hardware or after SDK changes.
