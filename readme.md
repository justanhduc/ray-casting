# Ray casting

A CPU + GPU implementations of ray casting.

## Dependencies

OpenCV

Eigen

CUDA (tested with CUDA 11.0)

## Building

Firstly, setup paths for OpenCV, Eigen and CUDA.

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

## References

The implementations are heavily borrowed from [here](https://github.com/Scoobadood/TSDF)
