#ifndef RenderUtilitiesGPU_h
#define RenderUtilitiesGPU_h

#include "cuda_utilities.cuh"
#include "PngWrapper.hpp"
#include "RenderUtilities.hpp"

#include <Eigen/Dense>

class Camera;

void render_scene(uint16_t width, uint16_t height, float3 *vertices, float3 *normals,
                  const Eigen::Vector3f & light_source, uint8_t *image);

#endif // RenderUtilities_h
