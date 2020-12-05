# Ray casting

A simple CPU + GPU implementations of ray casting with anti-aliasing.

## Dependencies

OpenCV

Eigen

CUDA (tested with CUDA 11.0)

## Building

Firstly, setup paths for OpenCV, Eigen and CUDA.

For Eigen, set an `EIGEN` flag to point to the root of the Eigen library.

For CUDA, set a `CUDA_HOME` flag to point to the root of the CUDA library.

Next, execute

```
cmake -DCMAKE_BUILD_TYPE=Debug .
```

for debug configuration or 

```
cmake -DCMAKE_BUILD_TYPE=Release .
```

for release configuration.

Then

```
make -j 8
```

to build the project.

## Data
The compatible data is a 1D array containing 3D TSDF values,
with all dimensions being vectorized and saved into a binary file.

## References

The implementations are heavily borrowed from [here](https://github.com/Scoobadood/TSDF)
