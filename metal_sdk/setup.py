from setuptools import setup, Extension, find_packages
import os
import pybind11
import platform

current_dir = os.path.dirname(os.path.abspath(__file__))

# Native SDK library path
sdk_lib = os.path.join(current_dir, "metal_sdk", "lib")

# Select the native library by CPU architecture.
arch = platform.machine()
if arch in ['x86_64', 'AMD64']:
    lib_path = os.path.join(sdk_lib, "x64")
    lib_name = "metal_sdk_x64"
elif arch in ['aarch64', 'arm64']:
    lib_path = os.path.join(sdk_lib, "arm64")
    lib_name = "metal_sdk_arm64"
else:
    raise RuntimeError(f"Unsupported architecture: {arch}")

ext_modules = [
    Extension(
        "metal_sdk.metal_sdk",   # Python import module name
        ["metal_sdk_interface.cpp"],
        include_dirs=[
            pybind11.get_include(),
            current_dir,
        ],
        library_dirs=[lib_path],
        libraries=[lib_name, 'glog'],
        language="c++",
        extra_compile_args=["-O3", "-std=c++14"],
        runtime_library_dirs=["$ORIGIN/lib/x64", "$ORIGIN/lib/arm64"],
    )
]

setup(
    name="metal_sdk",
    version="0.1.0",
    author="xiaofanZhang",
    description="Python SDK wrapper for the MakerMods Metal arm",
    packages=find_packages(),
    ext_modules=ext_modules,
    zip_safe=False,
    install_requires=[
        "pybind11>=2.10.0",
    ],
    include_package_data=True,
    package_data={
        "metal_sdk": ["lib/x64/*.so", "lib/arm64/*.so"],
    },
    python_requires=">=3.7",
)
