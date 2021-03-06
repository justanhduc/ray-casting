//
//  Raycaster.hpp
//  KinFu
//
//  Created by Dave on 2/06/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef Raycaster_hpp
#define Raycaster_hpp

#include <Eigen/Core>

#include "TSDFVolumeCPU.hpp"
#include "Camera.hpp"
#include "DepthImage.hpp"

class Raycaster {
    public:
        explicit Raycaster( int width=640, int height=480) {
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
        float3cpu * compute_normals(const float3cpu * vertices) const;
        float3cpu * get_vertices(const TSDFVolumeCPU& volume, const Camera& camera) const;
        void raycast( const TSDFVolumeCPU & volume, const Camera & camera, Eigen::Matrix<float, 3, Eigen::Dynamic> &  vertices, Eigen::Matrix<float, 3, Eigen::Dynamic> &  normals ) const;
        void render_to_depth_image( const TSDFVolumeCPU & volume, const Camera & camera ) const;

    protected:

        uint16_t    m_width;
        uint16_t    m_height;

    };
#endif /* Raycaster_hpp */
