# Building the Python SDK

Install build dependencies:

```bash
sudo apt install -y pybind11-dev python3-pybind11
pip install pybind11
```

Build and install:

```bash
mkdir build
cd build
cmake ..
make
make install
```

Temporarily add the package to `PYTHONPATH`:

```bash
export PYTHONPATH="${PYTHONPATH}:/path/to/metal-python-ros2/metal_sdk"
python3 -c "import metal_sdk; print('Success!')"
```

Build a Python distribution package:

```bash
pip install build twine
python3 -m build
python3 -m build --sdist
```

Upload to PyPI, if you are publishing a release:

```bash
python3 -m twine upload dist/*
```

Install-test the package:

```bash
pip install metal_sdk
```
