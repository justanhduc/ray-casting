//
//  Raycaster.hpp
//  KinFu
//
//  Created by Dave on 2/06/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef GPURaycaster_hpp
#define GPURaycaster_hpp

#include <Eigen/Core>

#include "TSDFVolume.hpp"
#include "DepthImage.hpp"


class GPURaycaster {
public:
    explicit GPURaycaster( int width=640, int height=480) {
        m_width = width;
        m_height = height;
    }

    /**
     * Raycast the TSDF and store discovered vertices and normals in the ubput arrays
     * @param volume The volume to cast
     * @param camera The camera
     * @param vertices The vertices discovered
     * @param normals The normals
     */
    void raycast( const TSDFVolume & volume, const Camera & camera,
                          Eigen::Matrix<float, 3, Eigen::Dynamic> & vertices,
                          Eigen::Matrix<float, 3, Eigen::Dynamic> & normals ) const;

    /**
     * Render a depth image from a TSDF
     * @param volume The volume to cast
     * @param camera The camera
     * @return The DepthImage
     */
    DepthImage * render_to_depth_image( const TSDFVolume & volume, const Camera & camera ) const;

    void render_with_shading(const TSDFVolume & volume, const Camera & camera,
                             Eigen::Matrix<float, 3, Eigen::Dynamic> & vertices,
                             Eigen::Matrix<float, 3, Eigen::Dynamic> & normals,
                             const Eigen::Vector3f &light_source, int n_samples, uint8_t *image) const;

protected:

    uint16_t    m_width;
    uint16_t    m_height;
};
#endif /* GPURaycaster_hpp */
