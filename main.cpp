//  justanhduc
// Oct 2020

#include <Eigen/Core>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TSDFVolumeCPU.hpp"
#include "Raycaster.hpp"
#include "RenderUtilities.hpp"


#define MAX_PATH 260  // for linux


int main() {
    using namespace Eigen;

    const int dim_x = 101, dim_y = 139, dim_z = 106;
    const int n_max_frame = 1;
    auto *tsdf = new float[dim_x * dim_y * dim_z * n_max_frame];
    char stmp[MAX_PATH];
    float ftemp[8];
    for (int i = 0; i < n_max_frame; i++) {
        sprintf(stmp, "../volume/_tsdf_multi_%03d.bin", i);  // for linux
        FILE *fp = nullptr;
        fp = fopen(stmp, "rb");  // for linux
        if (fp == 0) {
            printf("Cannot read file %s\n", stmp);
            exit(-1);
        }
        fread(ftemp, sizeof(float), 8, fp);
        fread(tsdf + (dim_x * dim_y * dim_z) * i, dim_x * dim_y * dim_z, sizeof(float), fp);
        fclose(fp);
    }

    TSDFVolumeCPU volume(dim_x, dim_y, dim_z, dim_x * .01, dim_y * .01, dim_z * .01);
    volume.set_distance_data(tsdf);
    volume.set_truncation_distance(.09);
    std::cout << "Read file. Rendering." << std::endl;

    uint16_t width = 1024;
    uint16_t height = 768;

    Eigen::Matrix<float, 3, Eigen::Dynamic> vertices;
    Eigen::Matrix<float, 3, Eigen::Dynamic> normals;

    Vector3f light_source{0, 3, 3};
    auto eye = Vector3f{0.5, 0, 2};  // view point
    Camera cam((float) width / 2, (float) height / 2, (float) (width - 1) / 2,
               (float) (height - 1) / 2);
    cam.move_to(eye);
    cam.look_at(0.5, 0.5, 0.5);

    Raycaster r{width, height};
    r.raycast(volume, cam, vertices, normals);
    uint8_t *scene = render_scene(width, height, vertices, normals, cam, light_source);

    cv::Mat frame(height, width, 0, scene);
    cv::imshow("Frame", frame);
    cv::waitKey(0);
    return 0;
}

