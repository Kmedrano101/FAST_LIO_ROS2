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

#include <pcl_conversions/pcl_conversions.h>
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

using pointcloud_colorizer::Camera;
using pointcloud_colorizer::facing_to_rotation;

class PointCloudColorizerNode : public rclcpp::Node
{
public:
    PointCloudColorizerNode()
        : Node("pointcloud_colorizer")
    {
        // Global parameters
        declare_parameter("cloud_topic", "/cloud_registered_body");
        declare_parameter("output_topic", "/cloud_registered_rgb");
        declare_parameter("num_cameras", 2);
        declare_parameter("use_compressed", true);
        declare_parameter("base_to_body_t", std::vector<double>{0.011, -0.06588, -0.02329});
        declare_parameter("min_depth", 0.5);
        declare_parameter("max_depth", 30.0);
        declare_parameter("image_margin", 5);
        declare_parameter("intensity_as_grayscale", true);
        declare_parameter("odom_topic", "/Odometry");
        declare_parameter("output_frame", "camera_init");
        declare_parameter("pcd_save_en", false);
        declare_parameter("pcd_file_path", std::string("~/ros2_ws/src/fast_lio_ros2/PCD/colorized_map.pcd"));
        declare_parameter("pcd_voxel_size", 0.05);

        auto cloud_topic = get_parameter("cloud_topic").as_string();
        auto output_topic = get_parameter("output_topic").as_string();
        auto odom_topic = get_parameter("odom_topic").as_string();
        output_frame_ = get_parameter("output_frame").as_string();
        int num_cameras = get_parameter("num_cameras").as_int();
        use_compressed_ = get_parameter("use_compressed").as_bool();
        auto base_to_body_vec = get_parameter("base_to_body_t").as_double_array();
        min_depth_ = get_parameter("min_depth").as_double();
        max_depth_ = get_parameter("max_depth").as_double();
        int margin = get_parameter("image_margin").as_int();
        intensity_fallback_ = get_parameter("intensity_as_grayscale").as_bool();

        pcd_save_en_ = get_parameter("pcd_save_en").as_bool();
        pcd_file_path_ = get_parameter("pcd_file_path").as_string();
        pcd_voxel_size_ = get_parameter("pcd_voxel_size").as_double();

        if (pcd_save_en_) {
            accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            RCLCPP_INFO(get_logger(), "PCD accumulation ENABLED (voxel=%.3f m)", pcd_voxel_size_);
        }

        Eigen::Vector3d base_to_body_t(base_to_body_vec[0], base_to_body_vec[1], base_to_body_vec[2]);

        // Setup cameras
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

            // Camera position in base_link → convert to body frame (L1 IMU body)
            Eigen::Vector3d pos_base(pos_vec[0], pos_vec[1], pos_vec[2]);
            state->camera.t = pos_base + base_to_body_t;

            // Rotation from body frame to camera frame
            state->camera.R = facing_to_rotation(facing);

            // Subscribe to camera image
            if (use_compressed_) {
                state->compressed_sub = create_subscription<sensor_msgs::msg::CompressedImage>(
                    image_topic,
                    rclcpp::SensorDataQoS(),
                    [this, idx = cameras_.size()](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                        compressed_image_callback(idx, msg);
                    });
            } else {
                state->raw_sub = create_subscription<sensor_msgs::msg::Image>(
                    image_topic,
                    rclcpp::SensorDataQoS(),
                    [this, idx = cameras_.size()](const sensor_msgs::msg::Image::SharedPtr msg) {
                        image_callback(idx, msg);
                    });
            }

            RCLCPP_INFO(get_logger(), "Camera %d: topic=%s facing=%s compressed=%s pos_body=[%.3f, %.3f, %.3f]",
                        i, image_topic.c_str(), facing.c_str(),
                        use_compressed_ ? "true" : "false",
                        state->camera.t.x(), state->camera.t.y(), state->camera.t.z());

            cameras_.push_back(std::move(state));
        }

        // Subscribe to point cloud
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloudColorizerNode::cloud_callback, this, std::placeholders::_1));

        // Publisher for colored point cloud
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        // Subscribe to odometry for body->global transform
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&PointCloudColorizerNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Initialized with %zu cameras", cameras_.size());
        RCLCPP_INFO(get_logger(), "  cloud: %s -> %s (frame: %s)",
                    cloud_topic.c_str(), output_topic.c_str(), output_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  odom: %s", odom_topic.c_str());
        RCLCPP_INFO(get_logger(), "  depth: [%.1f, %.1f] m, margin: %d px",
                    min_depth_, max_depth_, margin);
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
            if (decoded.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Camera %zu: imdecode returned empty image", cam_idx);
                return;
            }
            std::lock_guard<std::mutex> lock(state.mutex);
            state.latest_image = decoded;
            state.camera.width = decoded.cols;
            state.camera.height = decoded.rows;
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Camera %zu compressed decode error: %s", cam_idx, e.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_R_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
        odom_t_ = Eigen::Vector3d(p.x, p.y, p.z);
        has_odom_ = true;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Grab latest images from all cameras
        std::vector<cv::Mat> images(cameras_.size());
        std::vector<bool> has_image(cameras_.size(), false);

        for (size_t i = 0; i < cameras_.size(); ++i) {
            std::lock_guard<std::mutex> lock(cameras_[i]->mutex);
            if (!cameras_[i]->latest_image.empty()) {
                images[i] = cameras_[i]->latest_image.clone();
                has_image[i] = true;
            }
        }

        // Check if at least one camera has an image
        bool any_image = false;
        for (bool h : has_image) { if (h) { any_image = true; break; } }
        if (!any_image) {
            if (cloud_count_ == 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "No camera images yet, waiting...");
            }
            return;
        }

        // Parse input point cloud (PointXYZINormal)
        pcl::PointCloud<pcl::PointXYZINormal> input_cloud;
        try {
            pcl::fromROSMsg(*msg, input_cloud);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Failed to parse cloud: %s", e.what());
            return;
        }

        if (input_cloud.empty()) return;

        // Grab latest odometry (body pose in global frame)
        Eigen::Matrix3d R_odom;
        Eigen::Vector3d t_odom;
        bool transform_to_global = false;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (has_odom_ && !output_frame_.empty()) {
                R_odom = odom_R_;
                t_odom = odom_t_;
                transform_to_global = true;
            }
        }

        // Colorize
        pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
        output_cloud.header = input_cloud.header;
        output_cloud.resize(input_cloud.size());

        size_t colored_counts[cameras_.size()];
        for (size_t i = 0; i < cameras_.size(); ++i) colored_counts[i] = 0;
        size_t uncolored = 0;

        for (size_t i = 0; i < input_cloud.size(); ++i) {
            const auto& pin = input_cloud[i];
            auto& pout = output_cloud[i];

            Eigen::Vector3d p_body(pin.x, pin.y, pin.z);

            // Output in global frame if odometry is available
            if (transform_to_global) {
                Eigen::Vector3d p_global = R_odom * p_body + t_odom;
                pout.x = static_cast<float>(p_global.x());
                pout.y = static_cast<float>(p_global.y());
                pout.z = static_cast<float>(p_global.z());
            } else {
                pout.x = pin.x;
                pout.y = pin.y;
                pout.z = pin.z;
            }

            bool colored = false;
            for (size_t c = 0; c < cameras_.size(); ++c) {
                if (!has_image[c]) continue;

                const auto& cam = cameras_[c]->camera;
                Eigen::Vector2d pixel;
                double depth;

                if (cam.project(p_body, pixel, depth) &&
                    depth >= min_depth_ && depth <= max_depth_)
                {
                    Eigen::Vector3f bgr = Camera::bilinear_sample(
                        images[c], pixel.x(), pixel.y());

                    // BGR -> RGB for PCL PointXYZRGB
                    pout.r = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, bgr[2])));
                    pout.g = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, bgr[1])));
                    pout.b = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, bgr[0])));

                    colored = true;
                    colored_counts[c]++;
                    break;
                }
            }

            if (!colored) {
                if (intensity_fallback_) {
                    uint8_t gray = static_cast<uint8_t>(
                        std::min(255.0f, std::max(0.0f, pin.intensity)));
                    pout.r = pout.g = pout.b = gray;
                } else {
                    pout.r = pout.g = pout.b = 128;
                }
                uncolored++;
            }
        }

        // Publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(output_cloud, output_msg);
        output_msg.header = msg->header;
        if (transform_to_global) {
            output_msg.header.frame_id = output_frame_;
        }
        cloud_pub_->publish(output_msg);

        // Accumulate for PCD save
        if (pcd_save_en_) {
            std::lock_guard<std::mutex> lock(accum_mutex_);
            *accumulated_cloud_ += output_cloud;
        }

        // Periodic stats
        cloud_count_++;
        if (cloud_count_ % 100 == 1) {
            std::string stats = "Colorized cloud #" + std::to_string(cloud_count_) +
                                ": " + std::to_string(input_cloud.size()) + " pts";
            for (size_t c = 0; c < cameras_.size(); ++c) {
                stats += " | cam" + std::to_string(c) + "=" + std::to_string(colored_counts[c]);
            }
            stats += " | uncolored=" + std::to_string(uncolored);
            RCLCPP_INFO(get_logger(), "%s", stats.c_str());
        }
    }

    std::vector<std::unique_ptr<CameraState>> cameras_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // Odometry: body pose in global (camera_init) frame
    std::mutex odom_mutex_;
    Eigen::Matrix3d odom_R_{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d odom_t_{Eigen::Vector3d::Zero()};
    bool has_odom_{false};
    std::string output_frame_;

    double min_depth_;
    double max_depth_;
    bool intensity_fallback_;
    bool use_compressed_;
    uint64_t cloud_count_{0};

    // PCD accumulation
    bool pcd_save_en_{false};
    std::string pcd_file_path_;
    double pcd_voxel_size_{0.05};
    std::mutex accum_mutex_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;

public:
    void save_pcd()
    {
        if (!pcd_save_en_ || !accumulated_cloud_) return;

        std::lock_guard<std::mutex> lock(accum_mutex_);
        if (accumulated_cloud_->empty()) {
            RCLCPP_WARN(get_logger(), "No colorized points accumulated, nothing to save");
            return;
        }

        // Expand ~ in path
        std::string path = pcd_file_path_;
        if (!path.empty() && path[0] == '~') {
            const char* home = std::getenv("HOME");
            if (home) path = std::string(home) + path.substr(1);
        }

        // Add timestamp to filename
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&t, &tm_buf);
        char ts[32];
        std::strftime(ts, sizeof(ts), "_%Y%m%d_%H%M%S", &tm_buf);

        // Insert timestamp before .pcd extension
        auto dot = path.rfind(".pcd");
        if (dot != std::string::npos) {
            path.insert(dot, ts);
        } else {
            path += ts + std::string(".pcd");
        }

        // Ensure parent directory exists
        std::filesystem::create_directories(std::filesystem::path(path).parent_path());

        RCLCPP_INFO(get_logger(), "Accumulated %zu raw points, applying voxel filter (%.3f m)...",
                    accumulated_cloud_->size(), pcd_voxel_size_);

        // Voxel downsample
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        if (pcd_voxel_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(accumulated_cloud_);
            vg.setLeafSize(pcd_voxel_size_, pcd_voxel_size_, pcd_voxel_size_);
            vg.filter(*filtered);
        } else {
            filtered = accumulated_cloud_;
        }

        RCLCPP_INFO(get_logger(), "Saving colorized PCD: %zu points -> %s",
                    filtered->size(), path.c_str());
        pcl::io::savePCDFileBinary(path, *filtered);
        RCLCPP_INFO(get_logger(), "Colorized PCD saved successfully!");
    }
};

static std::shared_ptr<PointCloudColorizerNode> g_node = nullptr;

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
    g_node = std::make_shared<PointCloudColorizerNode>();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::spin(g_node);
    // Save if spin exits normally (signal handler already saved + reset if triggered)
    if (g_node) {
        g_node->save_pcd();
        g_node.reset();
    }
    rclcpp::shutdown();
    return 0;
}
