//
// Created by justanhduc on 10/24/20.
//

#ifndef RAY_CASTING_TSDFUTILITIESCPU_H
#define RAY_CASTING_TSDFUTILITIESCPU_H

#include <cstdint>
#include <cmath>

#include "TSDFVolumeCPU.hpp"

/**
* @param x The horizontal coord (0-width-1)
* @param y The vertical coord (0-height - 1)
* @param z The depth coord (0-depth - 1)
* @return The coordinate of the centre of the given voxel in world coords (mm)
*/
float3cpu centre_of_voxel_at( int x, int y, int z, const float3cpu& voxel_size, const float3cpu& offset=float3cpu{0.0f, 0.0f, 0.0f});


/**
 * @param x The voxel x coord
 * @param y The voxel y coord
 * @param z The voxel z coord
 * @param tsdf_values The values
 * @param voxel_grid_size The size of the voxel grid
 * @return the value at voxel (x,y,z)
 */
float tsdf_value_at( int x, int y, int z, const float * tsdf_values, const dim3cpu voxel_grid_size );

/**
 * Determine the voxel in which a point lies
 * @param point The point in voxel space coordinates (0,0,0) -> (max_x, max_y, max_z)
 * @return The voxel in which the point lies.
 */
int3cpu voxel_for_point( const float3cpu point, const float3cpu voxel_size );



#endif //RAY_CASTING_TSDFUTILITIESCPU_H
