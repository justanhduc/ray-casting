//
//  GPUTSDFVolume.hpp
//  A GPU Based TSDF
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef TSDFVolumeCPU_hpp
#define TSDFVolumeCPU_hpp

#include "Camera.hpp"

//#include <Eigen/Core>

#include <cstdint>
#include <iostream>
#include <string>
#include <cmath>


typedef struct {
    float m11, m21, m31, m41;
    float m12, m22, m32, m42;
    float m13, m23, m33, m43;
    float m14, m24, m34, m44;
} Mat44CPU;

typedef struct {
    float m11, m21, m31;
    float m12, m22, m32;
    float m13, m23, m33;
} Mat33CPU;

struct float3cpu {
    float x, y, z;
};

struct int3cpu {
    int x, y, z;
};

struct dim3cpu {
    unsigned int x, y, z;
};

struct uchar3cpu {
    unsigned char x, y, z;
};

inline float f3_dot(const float3cpu& f1, const float3cpu& f2) {
    return f1.x * f2.x + f1.y * f2.y + f1.z * f2.z;
}

/**
 * Subtract one float3 from another
 * @param f1 First float3
 * @param f2 Second float3
 * @return f1 - f2
 */
inline float3cpu f3_sub( const float3cpu& f1, const float3cpu& f2 ) {
    float3cpu f{f1.x-f2.x, f1.y-f2.y, f1.z-f2.z};
    return f;
}

/**
 * Add one float3 to another
 * @param f1 First float3
 * @param f2 Second float3
 * @return f1 + f2
 */
inline float3cpu f3_add( const float3cpu& f1, const float3cpu& f2 ) {
    float3cpu f{f2.x+f1.x, f2.y+f1.y, f2.z+f1.z};
    return f;
}

/**
 * Multiply a float3 by a scalar value
 * @param s The scalar
 * @param vec The float3
 */
inline float3cpu f3_mul_scalar( const float& scalar, const float3cpu& vec ) {
    float3cpu f{vec.x * scalar, vec.y * scalar, vec.z * scalar};
    return f;
}


/**
 * Perform per element division of f1 by v2
 * @param f1 the first float3 (numerators)
 * @param f2 The second float3 (denominators)
 * @return f1 ./ f2
 */
inline float3cpu f3_div_elem(const float3cpu& f1, const float3cpu& f2 ) {
    float3cpu f{f1.x/f2.x, f1.y/f2.y, f1.z/f2.z};
    return f;
}

/**
 * Perform per element division of f1 by v2
 * @param f1 the first float3 (numerators)
 * @param i2 The second float3 (denominators)
 * @return f1 ./ f2
 */
inline float3cpu f3_div_elem( const float3cpu& f, const dim3cpu& i ) {
    float3cpu ff{f.x/i.x, f.y/i.y, f.z/i.z};
    return ff;
}

/**
 * Normalise a float3
 * @param vec The float3 vector
 */
inline float3cpu f3_normalise(float3cpu vec) {
    float l = sqrt( vec.x*vec.x+vec.y*vec.y+vec.z*vec.z);
    vec.x /= l;
    vec.y /= l;
    vec.z /= l;
    return vec;
}

/**
 * Normalise a float3
 * @param vec The float3 vector
 */
inline float f3_norm( float3cpu vec ) {
    return sqrt(vec.x*vec.x+vec.y*vec.y+vec.z*vec.z);
}

/**
 * Perform a matrix multiplication
 * @param mat33 The matrix
 * @param vec3 The vector
 * @return an output vector
 */
inline float3cpu m3_f3_mul( const Mat33CPU& mat, const float3cpu& vec ) {
    float3cpu result;
    result.x = mat.m11 * vec.x + mat.m12 * vec.y + mat.m13 * vec.z;
    result.y = mat.m21 * vec.x + mat.m22 * vec.y + mat.m23 * vec.z;
    result.z = mat.m31 * vec.x + mat.m32 * vec.y + mat.m33 * vec.z;
    return result;
}

/**
 * Perform a matrix multiplication
 * @param mat33 The matrix
 * @param vec3 The vector
 * @return an output vector
 */
inline float3cpu m3_i3_mul( const Mat33CPU& mat, const int3cpu& vec ) {
    float3cpu result;
    result.x = mat.m11 * vec.x + mat.m12 * vec.y + mat.m13 * vec.z;
    result.y = mat.m21 * vec.x + mat.m22 * vec.y + mat.m23 * vec.z;
    result.z = mat.m31 * vec.x + mat.m32 * vec.y + mat.m33 * vec.z;
    return result;
}

/**
 * Project down into pixel coordinates
 * given the inv_pose and K matrices
 * plus a point in world coords
 */
inline int3cpu world_to_pixel( const Mat44CPU & inv_pose, const Mat33CPU& k, const float3cpu& point ) {
    float3cpu cam_point;
    cam_point.x = inv_pose.m11 * point.x + inv_pose.m12 * point.y + inv_pose.m13 * point.z + inv_pose.m14;
    cam_point.y = inv_pose.m21 * point.x + inv_pose.m22 * point.y + inv_pose.m23 * point.z + inv_pose.m24;
    cam_point.z = inv_pose.m31 * point.x + inv_pose.m32 * point.y + inv_pose.m33 * point.z + inv_pose.m34;
    float w = inv_pose.m41 * point.x + inv_pose.m42 * point.y + inv_pose.m43 * point.z + inv_pose.m44;
    cam_point.x /= w;
    cam_point.y /= w;
    cam_point.z /= w;

    float3cpu image_point {cam_point.x / cam_point.z, cam_point.y / cam_point.z, 1.0f};
    int3cpu pixel;
    pixel.x = static_cast<uint>( round( k.m11 * image_point.y + k.m12 * image_point.z + k.m13 ) );
    pixel.y = static_cast<uint>( round( k.m21 * image_point.y + k.m22 * image_point.z + k.m23 ) );
    pixel.z = static_cast<uint>( round( k.m31 * image_point.y + k.m32 * image_point.z + k.m33 ) ); // Should be 1

    return pixel;
}

class TSDFVolumeCPU {
public:
    struct DeformationNode {
        float3cpu  translation;
        float3cpu  rotation;
    };

    struct Float3 {
        float x;
        float y;
        float z;
        inline Float3( const float3cpu& rhs ) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }
        inline Float3( float fx=0.0f, float fy=0.0f, float fz=0.0f ) {
            x = fx;
            y = fy;
            z = fz;
        }
        inline operator float3cpu() const {return float3cpu{x, y, z}; }

        inline Float3 operator -( const Float3& rhs ) const {
            return float3cpu{x - rhs.x, y - rhs.y, z - rhs.z};
        }
        inline Float3 operator +( const Float3& rhs ) const {
            return float3cpu{x + rhs.x, y + rhs.y, z + rhs.z};
        }
        inline Float3 operator /( const float scalar ) const {
            return float3cpu{x / scalar, y / scalar, z / scalar};
        }
        inline Float3 operator *( const Float3& rhs ) const {
            return float3cpu{x * rhs.x, y * rhs.y, z * rhs.z};
        }
        inline float norm( ) const {
            return std::sqrt( x*x + y*y + z*z );
        }

    };

    struct Int3 {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    struct  UInt3 {
        unsigned int x;
        unsigned int y;
        unsigned int z;

        inline UInt3( const dim3cpu& rhs ) : x{rhs.x}, y{rhs.y}, z{rhs.z} {};
        inline UInt3( uint32_t x, uint32_t y, uint32_t z ) : x{x}, y{y}, z{z} {};
        inline operator dim3cpu() const {return dim3cpu{x, y, z}; }
    };

    ~TSDFVolumeCPU();

    /**
    * Make a TSDFVolume with the given dimensions (voxels) and physcial dimensions
    * @param size The number of voxels in each X,Y and Z dimension
    * @param physical_size The size ( in mm ) of the space contained in the volume
    */
    TSDFVolumeCPU( const UInt3& size = UInt3{64, 64, 64},
                const Float3& physical_size = Float3 { 3000.0f, 3000.0f, 3000.0f} );


    /**
     * Make a TSDFVolume with the given dimensins and physical dimensions
     * @param volume_x X dimension in voxels
     * @param volume_y Y dimension in voxels
     * @param volume_z Z dimension in voxels
     * @param psize_x Physical size in X dimension in mm
     * @param psize_y Physical size in Y dimension in mm
     * @param psize_z Physical size in Z dimension in mm
     */
    TSDFVolumeCPU( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z );

    /**
     * Load a TSDFVolume from the specified file. The volume must previously have been saved
     */
    TSDFVolumeCPU( const std::string& file_name );

    /**
     * Set the size of the volume. This will delete any existing values and resize the volume, clearing it when done.
     * Volume offset is maintained
     * @param volume_x X dimension in voxels
     * @param volume_y Y dimension in voxels
     * @param volume_z Z dimension in voxels
     * @param psize_x Physical size in X dimension in mm
     * @param psize_y Physical size in Y dimension in mm
     * @param psize_z Physical size in Z dimension in mm
     */
    void set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z);

    void set_truncation_distance(float d);

    /**
     * @return the size of this space in voxels.
     */
    inline UInt3 size( ) const { return (UInt3) m_size; }

    /**
     * @return the dimensions of each voxel in mm
     */
    inline Float3 voxel_size( ) const { return (Float3) m_voxel_size; }

    /**
     * @return the physical size of the volume in world coords (mm)
     */
    inline Float3 physical_size( ) const { return (Float3) m_physical_size; }

    /**
     * @return the truncation distance (mm)
     */
    inline float truncation_distance( ) const { return m_truncation_distance; }

    /**
     * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
     * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
     * @param ox X offset in mm
     * @param oy Y offset in mm
     * @param oz Z offset in mm
     */
    inline void offset( float ox, float oy, float oz ) {
        m_offset.x = ox;
        m_offset.y = oy;
        m_offset.z = oz;
    }

    /**
     * @return the offset f the TSDF volume in space
     */
    inline Float3 offset( ) const { return (Float3) m_offset; }

    /**
     * Clear the voxel and weight data
     */
    void clear( );

#pragma mark - Data access

 /**
     *
     */
    inline size_t index( int x, int y, int z ) const {
        return x + (y * m_size.x) + (z * m_size.x * m_size.y);
    };

    /**
     * @return pointer to translation data
     */
    inline DeformationNode *  deformation() const {
        return m_deformation_nodes;
    }

    /**
     * Set the deformation data for this space
     * @param data Data in host memory space; Assumed to be vx*vy*vz DeformationNode
     */
    void set_deformation( DeformationNode *deformation);

    /**
     * Return pointer to distance data
     * @return Pointer to distance data
     */
    inline const float * distance_data() const {
        return m_distances;
    }

    /**
     * Set the distance data for the TSDF in one call
     * @param distance_data Pointer to enough floats to populate the TSFD
     */
    void set_distance_data(float * distance_data);


    /**
     * Return pointer to weight data
     * @return Pointer to weight data
     */
    inline const float * weight_data() const {
        return m_weights;
    }

    /**
     * Set the weight data for the TSDF in one call
     * @param weight_data Pointer to enough floats to populate the TSFD
     */
    void set_weight_data( const float * weight_data );


    /**
     * @return the global rotation of the TSDF deformation 
     * as a vector of 3 Euler angles (X then Y then Z - also Tait-Bryan angles)
     */
    inline float3cpu global_rotation( ) const {
        return m_global_rotation;
    }

    /**
     * @return the global translation of the TSDF deformation 
     */
    inline float3cpu global_translation( ) const {
        return m_global_translation;
    }


#pragma mark - Deform a set of points
    /**
     * Apply the volume's deformation field t the given set of points, modifying them in place
     */
    void deform_mesh( const int num_points, float3cpu * points ) const;

#pragma mark - Integrate new depth data
    /**
     * Integrate a range map into the TSDF
     * @param depth_map Pointer to width*height depth values where 0 is an invalid depth and positive values are expressed in mm
     * @param width The horiontal dimension of the depth_map
     * @param height The height of the depth_map
     * @param camera The camera from which the depth_map was taken
     */
    void integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera );

#pragma mark - Import/Export

    /**
     * Save the TSDF to file
     * @param The filename
     * @return true if the file saved OK otherwise false.
     */
    bool save_to_file( const std::string & file_name) const;

    /**
     * Load the given TSDF file
     * @param The filename
     * @return true if the file saved OK otherwise false.
     */
    bool load_from_file( const std::string & file_name);

#pragma mark - Rendering
    void raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const ;

private:
    /**
     * Deallocate storage for this TSDF
     */
    void deallocate( );

    // Size of voxel grid
    dim3cpu                    m_size;

    // Physical size of space represented in mm
    float3cpu                  m_physical_size;

    // Offset of physical grid in world coordinates
    float3cpu                  m_offset;

    // Size of a voxel
    float3cpu m_voxel_size;

    //  Truncation distance
    float m_truncation_distance;

    // Max weight for a voxel
    float m_max_weight;

    // Per grid point data
    float                   *m_distances;

    // Colour data, RGB as 3xuchar
    uchar3cpu                  *m_colours;

    //  Confidence weight for distance and colour
    float                   *m_weights;

    // Deformation field
    DeformationNode         *m_deformation_nodes;

    // Global translation
    float3cpu                  m_global_translation;

    // Global rotation
    float3cpu                  m_global_rotation;
};
#endif /* TSDFVolumeCPU_hpp */
