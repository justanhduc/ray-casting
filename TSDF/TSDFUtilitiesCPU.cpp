//
// Created by justanhduc on 10/24/20.
//

#include "TSDFUtilitiesCPU.h"

/**
* @param x The horizontal coord (0-width-1)
* @param y The vertical coord (0-height - 1)
* @param z The depth coord (0-depth - 1)
* @return The coordinate of the centre of the given voxel in world coords (mm)
*/
float3cpu centre_of_voxel_at(int x, int y, int z, const float3cpu& voxel_size, const float3cpu& offset)  {
    float3cpu centre {
            (x + 0.5f) * voxel_size.x + offset.x,
            (y + 0.5f) * voxel_size.y + offset.y,
            (z + 0.5f) * voxel_size.z + offset.z
    };
    return centre;
}


/**
 * @param x The voxel x coord
 * @param y The voxel y coord
 * @param z The voxel z coord
 * @param tsdf_values The values
 * @param voxel_grid_size The size of the voxel grid
 * @return the value at voxel (x,y,z)
 */
float tsdf_value_at(int x, int y, int z, const float * tsdf_values, const dim3cpu voxel_grid_size) {
    // Force out of bounds coords back in
    x = std::min(std::max(x, 0), (int) voxel_grid_size.x-1);
    y = std::min(std::max(y, 0), (int) voxel_grid_size.y-1);
    z = std::min(std::max(z, 0), (int) voxel_grid_size.z-1);

    size_t idx = voxel_grid_size.x * voxel_grid_size.y * z + voxel_grid_size.x * y + x;
    return tsdf_values[idx];
}

/**
 * Determine the voxel in which a point lies
 * @param point The point in voxel space coordinates (0,0,0) -> (max_x, max_y, max_z)
 * @return The voxel in which the point lies.
 */
int3cpu voxel_for_point( const float3cpu point, const float3cpu voxel_size ) {
    int3cpu voxel {
            int(std::floor( point.x / voxel_size.x )),
            int(std::floor( point.y / voxel_size.y )),
            int(std::floor( point.z / voxel_size.z ))
    };

    return voxel;
}