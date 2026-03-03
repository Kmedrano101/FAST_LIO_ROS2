#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>

namespace pointcloud_colorizer {

struct Camera {
    // Intrinsics
    double fx, fy, cx, cy;

    // Extrinsic: body frame -> camera frame
    // P_cam = R * (P_body - t)
    Eigen::Matrix3d R;   // rotation body->camera
    Eigen::Vector3d t;   // camera origin in body frame

    // Image dimensions (set when first image arrives)
    int width{0};
    int height{0};
    int margin{5};

    // Project a 3D body-frame point to pixel coordinates
    // Returns false if behind camera or outside image bounds
    bool project(const Eigen::Vector3d& p_body,
                 Eigen::Vector2d& pixel,
                 double& depth) const
    {
        Eigen::Vector3d p_cam = R * (p_body - t);
        depth = p_cam.z();

        if (depth <= 0.0) return false;

        double inv_z = 1.0 / depth;
        double u = fx * p_cam.x() * inv_z + cx;
        double v = fy * p_cam.y() * inv_z + cy;

        if (u < margin || u >= width - margin - 1 ||
            v < margin || v >= height - margin - 1)
            return false;

        pixel.x() = u;
        pixel.y() = v;
        return true;
    }

    // Bilinear interpolation for sub-pixel BGR sampling
    // Returns [B, G, R] as floats
    static Eigen::Vector3f bilinear_sample(const cv::Mat& img, double u, double v)
    {
        int u0 = static_cast<int>(u);
        int v0 = static_cast<int>(v);
        float du = static_cast<float>(u - u0);
        float dv = static_cast<float>(v - v0);

        float w00 = (1.0f - du) * (1.0f - dv);
        float w10 = du * (1.0f - dv);
        float w01 = (1.0f - du) * dv;
        float w11 = du * dv;

        const auto* row0 = img.ptr<cv::Vec3b>(v0);
        const auto* row1 = img.ptr<cv::Vec3b>(v0 + 1);

        const auto& p00 = row0[u0];
        const auto& p10 = row0[u0 + 1];
        const auto& p01 = row1[u0];
        const auto& p11 = row1[u0 + 1];

        return Eigen::Vector3f(
            w00 * p00[0] + w10 * p10[0] + w01 * p01[0] + w11 * p11[0],
            w00 * p00[1] + w10 * p10[1] + w01 * p01[1] + w11 * p11[1],
            w00 * p00[2] + w10 * p10[2] + w01 * p01[2] + w11 * p11[2]
        );
    }
};

// Compute body->camera rotation from cardinal facing direction
// Camera convention: X=right, Y=down, Z=forward (optical axis)
// Body convention (ROS): X=forward, Y=left, Z=up
inline Eigen::Matrix3d facing_to_rotation(const std::string& facing)
{
    Eigen::Matrix3d R;

    if (facing == "front") {
        // Optical axis = body +X
        // cam_X = body -Y, cam_Y = body -Z, cam_Z = body +X
        R <<  0, -1,  0,
              0,  0, -1,
              1,  0,  0;
    } else if (facing == "back") {
        // Optical axis = body -X
        // cam_X = body +Y, cam_Y = body -Z, cam_Z = body -X
        R <<  0,  1,  0,
              0,  0, -1,
             -1,  0,  0;
    } else if (facing == "left") {
        // Optical axis = body +Y
        // cam_X = body +X, cam_Y = body -Z, cam_Z = body +Y
        R <<  1,  0,  0,
              0,  0, -1,
              0,  1,  0;
    } else if (facing == "right") {
        // Optical axis = body -Y
        // cam_X = body -X, cam_Y = body -Z, cam_Z = body -Y
        R << -1,  0,  0,
              0,  0, -1,
              0, -1,  0;
    } else {
        R = Eigen::Matrix3d::Identity();
    }

    return R;
}

}  // namespace pointcloud_colorizer
