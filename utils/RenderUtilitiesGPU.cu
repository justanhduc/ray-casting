#include <iostream>
#include <Eigen/Core>

#include "RenderUtilitiesGPU.hpp"


__global__
void compute_shading(const float3 *vertices, const float3 *normals, const float3 light_source,
                     int width, int height, float amb_coeff, uint8_t *image) {
    const unsigned int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int idx_y = blockIdx.y * blockDim.y + threadIdx.y;

    if (idx_x > width - 1 || idx_y > height - 1)
        return;

    const size_t idx = idx_y * width + idx_x;
    float diff_coeff = 1. - amb_coeff;

    float3 r = f3_sub(light_source, vertices[idx]);  // vertex to lightsource direction
    f3_normalise(r);

    float shade = max(f3_dot(r, normals[idx]), 0.);
    shade = amb_coeff + (diff_coeff * shade);
    image[idx] = (uint8_t) floor(shade * 255.99);
}


void render_scene(uint16_t width, uint16_t height, float3 *vertices, float3 *normals,
                  const Eigen::Vector3f &light_source, uint8_t *image) {
    using namespace Eigen;

    auto ls = float3_from_eigen_vector(light_source);

    // allocate image data
    auto data_size = width * height;

    // Ensure that there's always ambient light
    float ambient_coefficient = .2;

    cudaError_t err;
    uint8_t *d_image = nullptr;
    err = cudaMalloc((void **) &d_image, data_size * sizeof(uint8_t));
    check_cuda_error("image alloc failed ", err);

    int block_dim = 32;
    dim3 grid_dim = dim3(divUp(width, block_dim), divUp(height, block_dim));
    compute_shading <<< grid_dim, dim3(block_dim, block_dim) >>>(
            vertices, normals, ls, width, height, ambient_coefficient, d_image);
    err = cudaDeviceSynchronize();
    check_cuda_error("failed to compute shading ", err);
    cudaMemcpy(image, d_image, data_size, cudaMemcpyDeviceToHost);
    cudaFree(d_image);
}

