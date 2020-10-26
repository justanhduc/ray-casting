//
//  main.cpp
//  VoxelSpaceViewer
//
//  Created by Dave on 26/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <Eigen/Core>
#include <cmath>

#include "TSDFVolume.hpp"
#include "GPURaycaster.hpp"
#include "RenderUtilities.hpp"
#include "opencv2/opencv.hpp"

#define MAX_PATH 260  // for linux


int main() {
    using namespace Eigen;

    const int dim_x = 101, dim_y = 139, dim_z = 106;
    const int n_max_frame = 1;
    const int width = 1024;
    const int height = 1024;

    GPURaycaster r{width, height};
    Vector3f light_source{0, 3, 3};
    auto eye = Vector3f{0.5, 0, 2};  // view point
    auto *cam = new Camera((float) width / 2, (float) height / 2, (float) (width - 1) / 2,
                           (float) (height - 1) / 2);
    cam->move_to(eye);
    cam->look_at(0.5, 0.5, 0.5);

    TSDFVolume volume(dim_x, dim_y, dim_z, dim_x * .01, dim_y * .01, dim_z * .01);
    volume.set_truncation_distance(.1);  // set low threshold as the step size may be large

    char stmp[MAX_PATH];
    float ftemp[8];
    for (int i = 0; i < n_max_frame; i++) {
        // reading tsdf files
        auto *tsdf = new float[dim_x * dim_y * dim_z * n_max_frame];
        sprintf(stmp, "../volume/_tsdf_multi_%03d.bin", i);  // for linux
        FILE *fp;
        fp = fopen(stmp, "rb");  // for linux
        if (fp == nullptr) {
            printf("Cannot read file %s\n", stmp);
            exit(-1);
        }
        fread(ftemp, sizeof(float), 8, fp);
        fread(tsdf + (dim_x * dim_y * dim_z) * i, dim_x * dim_y * dim_z, sizeof(float), fp);
        fclose(fp);

        // set tsdf to volume
        volume.set_distance_data(tsdf);

        // define containers for normals and vertices
        auto *vertices = new Eigen::Matrix<float, 3, Eigen::Dynamic>[width * height];
        auto *normals = new Eigen::Matrix<float, 3, Eigen::Dynamic>[width * height];

        // perform ray casting
        r.raycast(volume, *cam, *vertices, *normals);
//        r.render_to_depth_image(volume, *cam);
        uint8_t * scene = render_scene(width, height, *vertices, *normals, *cam, light_source);

        cv::Mat frame(height, width, 0, scene);
        cv::imshow("Frame", frame);
        auto c = (char) cv::waitKey(25);
        if (c == 27)
            break;
    }

    return 0;
}

