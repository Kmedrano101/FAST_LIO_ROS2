#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "pointcloud_colorizer/colorizer.hpp"

#include <csignal>
#include <mutex>
#include <vector>
#include <memory>
#include <chrono>
#include <filesystem>
#include <unordered_map>
#include <cfloat>
#include <atomic>

using pointcloud_colorizer::Camera;
using pointcloud_colorizer::facing_to_rotation;

// Spatial index key for world-frame voxel cells
struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        // Simple spatial hash
        size_t h = 73856093ULL * static_cast<size_t>(k.x) ^
                   19349663ULL * static_cast<size_t>(k.y) ^
                   83492791ULL * static_cast<size_t>(k.z);
        return h;
    }
};

class OfflineColorizerNode : public rclcpp::Node
{
public:
    OfflineColorizerNode()
        : Node("offline_colorizer")
    {
        // Parameters
        declare_parameter("pcd_input_path", std::string(""));
        declare_parameter("pcd_output_path", std::string("~/ros2_ws/src/fast_lio_ros2/PCD/colorized_full.pcd"));
        declare_parameter("frame_interval", 0.5);
        declare_parameter("spatial_cell_size", 2.0);
        declare_parameter("num_cameras", 2);
        declare_parameter("use_compressed", true);
        declare_parameter("base_to_body_t", std::vector<double>{0.011, -0.06588, -0.02329});
        declare_parameter("min_depth", 0.5);
        declare_parameter("max_depth", 30.0);
        declare_parameter("image_margin", 5);
        declare_parameter("intensity_as_grayscale", true);
        declare_parameter("odom_topic", "/Odometry");
        declare_parameter("pcd_voxel_size", 0.0);
        declare_parameter("idle_timeout", 5.0);

        auto pcd_input = get_parameter("pcd_input_path").as_string();
        pcd_output_ = get_parameter("pcd_output_path").as_string();
        frame_interval_ = get_parameter("frame_interval").as_double();
        cell_size_ = get_parameter("spatial_cell_size").as_double();
        int num_cameras = get_parameter("num_cameras").as_int();
        use_compressed_ = get_parameter("use_compressed").as_bool();
        auto base_to_body_vec = get_parameter("base_to_body_t").as_double_array();
        min_depth_ = get_parameter("min_depth").as_double();
        max_depth_ = get_parameter("max_depth").as_double();
        int margin = get_parameter("image_margin").as_int();
        intensity_fallback_ = get_parameter("intensity_as_grayscale").as_bool();
        auto odom_topic = get_parameter("odom_topic").as_string();
        pcd_voxel_size_ = get_parameter("pcd_voxel_size").as_double();
        double idle_timeout = get_parameter("idle_timeout").as_double();

        Eigen::Vector3d base_to_body_t(base_to_body_vec[0], base_to_body_vec[1], base_to_body_vec[2]);

        // ---- Load PCD ----
        if (pcd_input.empty()) {
            RCLCPP_FATAL(get_logger(), "pcd_input_path is required!");
            throw std::runtime_error("pcd_input_path not set");
        }
        // Expand ~
        if (!pcd_input.empty() && pcd_input[0] == '~') {
            const char* home = std::getenv("HOME");
            if (home) pcd_input = std::string(home) + pcd_input.substr(1);
        }

        RCLCPP_INFO(get_logger(), "Loading PCD: %s ...", pcd_input.c_str());
        world_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
        if (pcl::io::loadPCDFile(pcd_input, *world_cloud_) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to load PCD file: %s", pcd_input.c_str());
            throw std::runtime_error("Failed to load PCD");
        }
        num_points_ = world_cloud_->size();
        RCLCPP_INFO(get_logger(), "Loaded %zu points", num_points_);

        // ---- Allocate color storage ----
        rgb_data_.resize(num_points_ * 3, 0);
        is_colored_.resize(num_points_, false);
        best_depth_.resize(num_points_, FLT_MAX);

        // ---- Build spatial index ----
        RCLCPP_INFO(get_logger(), "Building spatial index (cell=%.1f m)...", cell_size_);
        double inv_cell = 1.0 / cell_size_;
        for (uint32_t i = 0; i < num_points_; ++i) {
            const auto& p = (*world_cloud_)[i];
            VoxelKey key{
                static_cast<int32_t>(std::floor(p.x * inv_cell)),
                static_cast<int32_t>(std::floor(p.y * inv_cell)),
                static_cast<int32_t>(std::floor(p.z * inv_cell))
            };
            spatial_index_[key].push_back(i);
        }
        RCLCPP_INFO(get_logger(), "Spatial index: %zu cells", spatial_index_.size());

        // ---- Setup cameras ----
        for (int i = 0; i < num_cameras; ++i) {
            std::string prefix = "camera_" + std::to_string(i);

            declare_parameter(prefix + ".image_topic", "");
            declare_parameter(prefix + ".fx", 1440.0);
            declare_parameter(prefix + ".fy", 1440.0);
            declare_parameter(prefix + ".cx", 960.0);
            declare_parameter(prefix + ".cy", 540.0);
            declare_parameter(prefix + ".position", std::vector<double>{0.0, 0.0, 0.0});
            declare_parameter(prefix + ".facing", "front");

            auto image_topic = get_parameter(prefix + ".image_topic").as_string();
            double fx = get_parameter(prefix + ".fx").as_double();
            double fy = get_parameter(prefix + ".fy").as_double();
            double cx = get_parameter(prefix + ".cx").as_double();
            double cy = get_parameter(prefix + ".cy").as_double();
            auto pos_vec = get_parameter(prefix + ".position").as_double_array();
            auto facing = get_parameter(prefix + ".facing").as_string();

            if (image_topic.empty()) {
                RCLCPP_WARN(get_logger(), "Camera %d has no image_topic, skipping", i);
                continue;
            }

            auto state = std::make_unique<CameraState>();
            state->camera.fx = fx;
            state->camera.fy = fy;
            state->camera.cx = cx;
            state->camera.cy = cy;
            state->camera.margin = margin;

            Eigen::Vector3d pos_base(pos_vec[0], pos_vec[1], pos_vec[2]);
            state->camera.t = pos_base + base_to_body_t;
            state->camera.R = facing_to_rotation(facing);

            if (use_compressed_) {
                state->compressed_sub = create_subscription<sensor_msgs::msg::CompressedImage>(
                    image_topic, rclcpp::SensorDataQoS(),
                    [this, idx = cameras_.size()](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                        compressed_image_callback(idx, msg);
                    });
            } else {
                state->raw_sub = create_subscription<sensor_msgs::msg::Image>(
                    image_topic, rclcpp::SensorDataQoS(),
                    [this, idx = cameras_.size()](const sensor_msgs::msg::Image::SharedPtr msg) {
                        image_callback(idx, msg);
                    });
            }

            RCLCPP_INFO(get_logger(), "Camera %d: topic=%s facing=%s", i, image_topic.c_str(), facing.c_str());
            cameras_.push_back(std::move(state));
        }

        // ---- Subscribe to odometry ----
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&OfflineColorizerNode::odom_callback, this, std::placeholders::_1));

        // ---- Idle timer: detect when bag replay finishes ----
        last_odom_time_ = std::chrono::steady_clock::now();
        idle_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(idle_timeout * 1000)),
            std::bind(&OfflineColorizerNode::idle_check, this));

        RCLCPP_INFO(get_logger(), "=== Offline Colorizer Ready ===");
        RCLCPP_INFO(get_logger(), "  %zu cameras, frame_interval=%.2fs", cameras_.size(), frame_interval_);
        RCLCPP_INFO(get_logger(), "  Waiting for bag replay (odom + images)...");
    }

private:
    struct CameraState {
        Camera camera;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub;
        cv::Mat latest_image;
        std::mutex mutex;
    };

    void image_callback(size_t cam_idx, const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (cam_idx >= cameras_.size()) return;
        auto& state = *cameras_[cam_idx];
        try {
            auto cv_img = cv_bridge::toCvCopy(msg, "bgr8");
            std::lock_guard<std::mutex> lock(state.mutex);
            state.latest_image = cv_img->image;
            state.camera.width = cv_img->image.cols;
            state.camera.height = cv_img->image.rows;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Camera %zu cv_bridge error: %s", cam_idx, e.what());
        }
    }

    void compressed_image_callback(size_t cam_idx,
                                    const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (cam_idx >= cameras_.size()) return;
        auto& state = *cameras_[cam_idx];
        try {
            cv::Mat decoded = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (decoded.empty()) return;
            std::lock_guard<std::mutex> lock(state.mutex);
            state.latest_image = decoded;
            state.camera.width = decoded.cols;
            state.camera.height = decoded.rows;
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Camera %zu decode error: %s", cam_idx, e.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_odom_time_ = std::chrono::steady_clock::now();
        started_ = true;

        // Time gating: skip frames too close together
        double stamp = rclcpp::Time(msg->header.stamp).seconds();
        if (stamp - last_processed_stamp_ < frame_interval_) return;
        last_processed_stamp_ = stamp;

        // Grab latest images
        std::vector<cv::Mat> images(cameras_.size());
        std::vector<bool> has_image(cameras_.size(), false);
        for (size_t i = 0; i < cameras_.size(); ++i) {
            std::lock_guard<std::mutex> lock(cameras_[i]->mutex);
            if (!cameras_[i]->latest_image.empty()) {
                images[i] = cameras_[i]->latest_image.clone();
                has_image[i] = true;
            }
        }
        bool any_image = false;
        for (bool h : has_image) { if (h) { any_image = true; break; } }
        if (!any_image) return;

        // Extract body pose in world frame: p_world = R_wb * p_body + t_wb
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        Eigen::Matrix3d R_wb = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
        Eigen::Vector3d t_wb(p.x, p.y, p.z);

        // Inverse: body = R_wb^T * (world - t_wb)
        Eigen::Matrix3d R_bw = R_wb.transpose();
        Eigen::Vector3d t_bw = -R_bw * t_wb;

        // Compute AABB of camera frustum in world frame
        // For each camera, get frustum corners in body frame, transform to world
        int32_t cell_min_x = INT32_MAX, cell_min_y = INT32_MAX, cell_min_z = INT32_MAX;
        int32_t cell_max_x = INT32_MIN, cell_max_y = INT32_MIN, cell_max_z = INT32_MIN;

        double inv_cell = 1.0 / cell_size_;

        for (size_t c = 0; c < cameras_.size(); ++c) {
            if (!has_image[c]) continue;
            const auto& cam = cameras_[c]->camera;

            // Frustum corners in camera frame at min and max depth
            // Camera convention: Z = optical axis
            double depths[] = {min_depth_, max_depth_};
            for (double d : depths) {
                // Image corners -> camera frame 3D
                double corners_u[] = {0.0, static_cast<double>(cam.width)};
                double corners_v[] = {0.0, static_cast<double>(cam.height)};
                for (double u : corners_u) {
                    for (double v : corners_v) {
                        // Unproject: p_cam = d * [(u-cx)/fx, (v-cy)/fy, 1]
                        Eigen::Vector3d p_cam(d * (u - cam.cx) / cam.fx,
                                              d * (v - cam.cy) / cam.fy,
                                              d);
                        // Camera frame -> body frame: p_body = R^T * p_cam + t
                        Eigen::Vector3d p_body = cam.R.transpose() * p_cam + cam.t;
                        // Body -> world
                        Eigen::Vector3d p_world = R_wb * p_body + t_wb;

                        int32_t cx_ = static_cast<int32_t>(std::floor(p_world.x() * inv_cell));
                        int32_t cy_ = static_cast<int32_t>(std::floor(p_world.y() * inv_cell));
                        int32_t cz_ = static_cast<int32_t>(std::floor(p_world.z() * inv_cell));

                        cell_min_x = std::min(cell_min_x, cx_);
                        cell_min_y = std::min(cell_min_y, cy_);
                        cell_min_z = std::min(cell_min_z, cz_);
                        cell_max_x = std::max(cell_max_x, cx_);
                        cell_max_y = std::max(cell_max_y, cy_);
                        cell_max_z = std::max(cell_max_z, cz_);
                    }
                }
            }
            // Also include camera position
            Eigen::Vector3d cam_world = R_wb * cam.t + t_wb;
            int32_t cx_ = static_cast<int32_t>(std::floor(cam_world.x() * inv_cell));
            int32_t cy_ = static_cast<int32_t>(std::floor(cam_world.y() * inv_cell));
            int32_t cz_ = static_cast<int32_t>(std::floor(cam_world.z() * inv_cell));
            cell_min_x = std::min(cell_min_x, cx_);
            cell_min_y = std::min(cell_min_y, cy_);
            cell_min_z = std::min(cell_min_z, cz_);
            cell_max_x = std::max(cell_max_x, cx_);
            cell_max_y = std::max(cell_max_y, cy_);
            cell_max_z = std::max(cell_max_z, cz_);
        }

        // Iterate over spatial cells within AABB
        size_t frame_colored = 0;
        size_t frame_checked = 0;

        for (int32_t ix = cell_min_x; ix <= cell_max_x; ++ix) {
            for (int32_t iy = cell_min_y; iy <= cell_max_y; ++iy) {
                for (int32_t iz = cell_min_z; iz <= cell_max_z; ++iz) {
                    VoxelKey key{ix, iy, iz};
                    auto it = spatial_index_.find(key);
                    if (it == spatial_index_.end()) continue;

                    for (uint32_t idx : it->second) {
                        frame_checked++;

                        const auto& pin = (*world_cloud_)[idx];
                        Eigen::Vector3d p_world(pin.x, pin.y, pin.z);

                        // World -> body
                        Eigen::Vector3d p_body = R_bw * p_world + t_bw;

                        // Try each camera
                        for (size_t c = 0; c < cameras_.size(); ++c) {
                            if (!has_image[c]) continue;

                            const auto& cam = cameras_[c]->camera;
                            Eigen::Vector2d pixel;
                            double depth;

                            if (cam.project(p_body, pixel, depth) &&
                                depth >= min_depth_ && depth <= max_depth_)
                            {
                                // Closest-distance update: only update if closer view
                                if (depth < best_depth_[idx]) {
                                    Eigen::Vector3f bgr = Camera::bilinear_sample(
                                        images[c], pixel.x(), pixel.y());

                                    size_t base = static_cast<size_t>(idx) * 3;
                                    rgb_data_[base + 0] = static_cast<uint8_t>(std::clamp(bgr[2], 0.0f, 255.0f)); // R
                                    rgb_data_[base + 1] = static_cast<uint8_t>(std::clamp(bgr[1], 0.0f, 255.0f)); // G
                                    rgb_data_[base + 2] = static_cast<uint8_t>(std::clamp(bgr[0], 0.0f, 255.0f)); // B

                                    best_depth_[idx] = static_cast<float>(depth);
                                    if (!is_colored_[idx]) {
                                        is_colored_[idx] = true;
                                        total_colored_++;
                                    }
                                    frame_colored++;
                                }
                                break; // First camera that sees it wins for this frame
                            }
                        }
                    }
                }
            }
        }

        frames_processed_++;

        // Log progress
        if (frames_processed_ % 5 == 1 || frame_colored > 10000) {
            double pct = 100.0 * total_colored_ / num_points_;
            RCLCPP_INFO(get_logger(),
                "Frame %zu: checked=%zu colored=%zu | Total: %zu/%zu (%.1f%%)",
                frames_processed_, frame_checked, frame_colored,
                total_colored_, num_points_, pct);
        }
    }

    void idle_check()
    {
        if (!started_) return;

        auto elapsed = std::chrono::steady_clock::now() - last_odom_time_;
        double sec = std::chrono::duration<double>(elapsed).count();

        if (sec > 3.0 && frames_processed_ > 0 && !saved_) {
            RCLCPP_INFO(get_logger(), "No odometry for %.1fs — bag replay appears finished.", sec);
            save_pcd();
        }
    }

    std::vector<std::unique_ptr<CameraState>> cameras_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr idle_timer_;

    // Loaded map
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_cloud_;
    size_t num_points_{0};

    // Color storage (separate arrays to avoid doubling memory with full XYZRGB cloud)
    std::vector<uint8_t> rgb_data_;       // R,G,B per point (3 * num_points)
    std::vector<bool> is_colored_;
    std::vector<float> best_depth_;
    size_t total_colored_{0};

    // Spatial index
    std::unordered_map<VoxelKey, std::vector<uint32_t>, VoxelKeyHash> spatial_index_;
    double cell_size_;

    // Config
    double min_depth_, max_depth_;
    double frame_interval_;
    double last_processed_stamp_{0.0};
    bool use_compressed_;
    bool intensity_fallback_;
    std::string pcd_output_;
    double pcd_voxel_size_;

    // State
    bool started_{false};
    bool saved_{false};
    size_t frames_processed_{0};
    std::chrono::steady_clock::time_point last_odom_time_;

public:
    void save_pcd()
    {
        if (saved_) return;
        saved_ = true;

        double pct = 100.0 * total_colored_ / num_points_;
        RCLCPP_INFO(get_logger(), "=== Saving colorized map ===");
        RCLCPP_INFO(get_logger(), "  Processed %zu frames, colored %zu/%zu points (%.1f%%)",
                    frames_processed_, total_colored_, num_points_, pct);

        // Build output XYZRGB cloud
        pcl::PointCloud<pcl::PointXYZRGB> output;
        output.resize(num_points_);

        for (size_t i = 0; i < num_points_; ++i) {
            const auto& in = (*world_cloud_)[i];
            auto& out = output[i];
            out.x = in.x;
            out.y = in.y;
            out.z = in.z;

            if (is_colored_[i]) {
                size_t base = i * 3;
                out.r = rgb_data_[base + 0];
                out.g = rgb_data_[base + 1];
                out.b = rgb_data_[base + 2];
            } else if (intensity_fallback_) {
                uint8_t gray = static_cast<uint8_t>(std::clamp(in.intensity, 0.0f, 255.0f));
                out.r = out.g = out.b = gray;
            } else {
                out.r = out.g = out.b = 128;
            }
        }

        // Expand ~ in output path
        std::string path = pcd_output_;
        if (!path.empty() && path[0] == '~') {
            const char* home = std::getenv("HOME");
            if (home) path = std::string(home) + path.substr(1);
        }

        // Add timestamp
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&t, &tm_buf);
        char ts[32];
        std::strftime(ts, sizeof(ts), "_%Y%m%d_%H%M%S", &tm_buf);

        auto dot = path.rfind(".pcd");
        if (dot != std::string::npos) {
            path.insert(dot, ts);
        } else {
            path += ts + std::string(".pcd");
        }

        std::filesystem::create_directories(std::filesystem::path(path).parent_path());

        // Optional voxel downsample
        if (pcd_voxel_size_ > 0.0) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(
                new pcl::PointCloud<pcl::PointXYZRGB>(output));
            pcl::PointCloud<pcl::PointXYZRGB> filtered;
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(cloud_ptr);
            vg.setLeafSize(pcd_voxel_size_, pcd_voxel_size_, pcd_voxel_size_);
            vg.filter(filtered);

            RCLCPP_INFO(get_logger(), "Voxel filter: %zu -> %zu points (%.3f m)",
                        output.size(), filtered.size(), pcd_voxel_size_);
            pcl::io::savePCDFileBinary(path, filtered);
        } else {
            pcl::io::savePCDFileBinary(path, output);
        }

        RCLCPP_INFO(get_logger(), "Saved: %s", path.c_str());
    }
};

static std::shared_ptr<OfflineColorizerNode> g_node = nullptr;

void signal_handler(int sig)
{
    (void)sig;
    if (g_node) {
        g_node->save_pcd();
        g_node.reset();
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<OfflineColorizerNode>();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::spin(g_node);
    if (g_node) {
        g_node->save_pcd();
        g_node.reset();
    }
    rclcpp::shutdown();
    return 0;
}
