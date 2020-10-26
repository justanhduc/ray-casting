//
// Created by justanhduc on 10/24/20.
//

#include "TSDFVolumeCPU.hpp"
//#include "TSDF_utilities.hpp"


/**
 * Constructor with specified number of voxels in each dimension
 * @param size
 * @param physical_size
 */
TSDFVolumeCPU::TSDFVolumeCPU( const UInt3& size, const Float3& physical_size ) : m_offset { 0.0, 0.0, 0.0 }, m_distances {NULL}, m_weights {NULL}, m_deformation_nodes{NULL}, m_colours{NULL} {
    if ( ( size.x > 0 ) && ( size.y > 0 ) && ( size.z > 0 ) &&
         ( physical_size.x > 0 ) && ( physical_size.y > 0 ) && ( physical_size.z > 0 ) ) {
        set_size( size.x, size.y, size.z , physical_size.x, physical_size.y, physical_size.z );
    } else {
        throw std::invalid_argument( "Attempt to construct TSDFVolume with zero or negative size" );
    }
}

/**
 * Make a TSDFVolume with the given dimensins and physical dimensions
 * @param volume_x X dimension in voxels
 * @param volume_y Y dimension in voxels
 * @param volume_z Z dimension in voxels
 * @param psize_x Physical size in X dimension in mm
 * @param psize_y Physical size in Y dimension in mm
 * @param psize_z Physical size in Z dimension in mm
 */
TSDFVolumeCPU::TSDFVolumeCPU( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z )  : m_offset { 0.0, 0.0, 0.0 }, m_distances {NULL}, m_weights {NULL}, m_deformation_nodes{NULL}, m_colours{NULL} {
    if ( ( volume_x > 0 ) && ( volume_y > 0 ) && ( volume_z > 0 ) &&
         ( psize_x > 0 ) && ( psize_y > 0 ) && ( psize_z > 0 ) ) {

        set_size( volume_x, volume_y, volume_z , psize_x, psize_y, psize_z );
    } else {
        throw std::invalid_argument( "Attempt to construct CPUTSDFVolume with zero or negative size" );
    }
}

/**
 * Set the distance data for the TSDF in one call
 * @param distance_data Pointer to enough floats to populate the TSFD
 */
void TSDFVolumeCPU::set_distance_data(float * distance_data) {
    m_distances = distance_data;
}

void TSDFVolumeCPU::set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z) {

    if ( ( volume_x != 0 && volume_y != 0 && volume_z != 0 ) && ( psize_x != 0 && psize_y != 0 && psize_z != 0 ) ) {
        m_size = dim3cpu { volume_x, volume_y, volume_z };
        m_physical_size = float3cpu { psize_x, psize_y, psize_z };

        // Compute truncation distance - must be at least 2x max voxel size
        m_voxel_size = f3_div_elem( m_physical_size, m_size );

        // Set t > diagonal of voxel
        m_truncation_distance = 1.1f * f3_norm( m_voxel_size );

        // Allocate device storage
        size_t data_size = volume_x * volume_y * volume_z;
        m_distances = new float [data_size];
        m_weights = new float [data_size];
        m_colours = new uchar3cpu [data_size];
//        m_deformation_nodes = new DeformationNode[data_size];

        m_global_rotation = float3cpu{0.0f, 0.0f, 0.0f};
        m_global_translation = float3cpu{0.0f, 0.0f, 0.0f};

        // Max weight for integrating depth images
        m_max_weight = 15.0f;

    } else {
        throw std::invalid_argument( "Attempt to set TSDF size or physical size to zero" );
    }
}

void TSDFVolumeCPU::set_truncation_distance(float d) {
    m_truncation_distance = d;
}

TSDFVolumeCPU::~TSDFVolumeCPU() {
    std::cout << "Destroying TSDFVolume" << std::endl;
    delete [] m_distances;
    delete [] m_colours;
    delete [] m_weights;
}
