import os
import platform
from setuptools import setup, Extension, find_packages
import pybind11

here = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.dirname(here)                       # src/metal_sdk
native_dir = os.path.join(pkg_root, "native")          # C++ sources + header
py_pkg = os.path.join(pkg_root, "metal_sdk")           # python package dir
lib_root = os.path.join(py_pkg, "lib")

arch = platform.machine()
if arch in ("x86_64", "AMD64"):
    lib_path, lib_name = os.path.join(lib_root, "x64"), "metal_sdk_x64"
elif arch in ("aarch64", "arm64"):
    lib_path, lib_name = os.path.join(lib_root, "arm64"), "metal_sdk_arm64"
else:
    raise RuntimeError(f"Unsupported architecture: {arch}")

ext_modules = [
    Extension(
        "metal_sdk.metal_sdk",
        ["wrapper.cpp"],
        include_dirs=[pybind11.get_include(), native_dir],
        library_dirs=[lib_path],
        libraries=[lib_name, "glog"],
        language="c++",
        extra_compile_args=["-O3", "-std=c++17"],
        runtime_library_dirs=["$ORIGIN/lib/x64", "$ORIGIN/lib/arm64"],
    )
]

setup(
    name="metal_sdk",
    version="0.1.0",
    author="MakerMods",
    description="Python SDK wrapper for the MakerMods Metal arm",
    package_dir={"metal_sdk": os.path.relpath(py_pkg, here)},
    packages=["metal_sdk"],
    ext_modules=ext_modules,
    zip_safe=False,
    install_requires=["pybind11>=2.9.0"],
    include_package_data=True,
    package_data={"metal_sdk": ["lib/x64/*.so", "lib/arm64/*.so"]},
    python_requires=">=3.10",
)
