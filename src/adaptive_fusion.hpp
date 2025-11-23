/**
 * @file adaptive_fusion.hpp
 * @brief Adaptive Multi-LiDAR Fusion Strategy for FAST-LIO
 *
 * This module implements an adaptive fusion strategy that dynamically switches
 * between asynchronous and bundle update modes based on feature density and
 * field-of-view coverage.
 *
 * @author Based on FAST_LIO_MULTI by engcang
 * @date 2025
 */

#ifndef ADAPTIVE_FUSION_HPP
#define ADAPTIVE_FUSION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <deque>
#include <mutex>

/**
 * @enum UpdateMode
 * @brief Defines the update strategies for multi-LiDAR fusion
 */
enum class UpdateMode : uint8_t {
    BUNDLE = 0,      ///< Merge all LiDAR scans before filter update
    ASYNC = 1,       ///< Update filter separately for each LiDAR scan
    ADAPTIVE = 2     ///< Dynamically switch between BUNDLE and ASYNC
};

/**
 * @class AdaptiveFusionManager
 * @brief Manages adaptive switching between bundle and async update modes
 *
 * This class monitors feature density and FOV coverage to determine the optimal
 * update strategy. It prevents frequent mode switching using hysteresis logic.
 */
class AdaptiveFusionManager {
public:
    /**
     * @brief Configuration parameters for adaptive fusion
     */
    struct Config {
        UpdateMode update_mode = UpdateMode::ADAPTIVE;  ///< Selected update mode
        int fov_threshold = 5000;                       ///< Min points to use ASYNC mode
        double feature_density_threshold = 0.002;       ///< Min feature density (points/m³)
        double hysteresis_ratio = 1.2;                  ///< Hysteresis factor to prevent oscillation
        int stability_frames = 3;                       ///< Frames to wait before mode switch
        bool enable_debug_output = false;               ///< Enable verbose logging
    };

    /**
     * @brief Default constructor with default configuration
     */
    AdaptiveFusionManager()
        : config_(),
          current_strategy_(UpdateMode::ASYNC),  // Start with ASYNC for lower latency
          frames_below_threshold_(0),
          frames_above_threshold_(0) {}

    /**
     * @brief Constructor with custom configuration
     * @param config Configuration parameters
     */
    explicit AdaptiveFusionManager(const Config& config)
        : config_(config),
          current_strategy_(UpdateMode::ASYNC),  // Start with ASYNC for lower latency
          frames_below_threshold_(0),
          frames_above_threshold_(0) {}

    /**
     * @brief Determine update strategy based on current scan characteristics
     * @param point_count Number of points in the current scan
     * @param scan_volume Estimated volume covered by the scan (m³)
     * @return Recommended update strategy
     */
    UpdateMode determineStrategy(int point_count, double scan_volume);

    /**
     * @brief Check if enough scans are available for bundle mode
     * @param buffer_size Current size of the scan buffer
     * @param num_lidars Number of active LiDAR sensors
     * @return True if bundle mode can proceed
     */
    bool canBundle(size_t buffer_size, int num_lidars) const;

    /**
     * @brief Get current update strategy
     * @return Current UpdateMode
     */
    UpdateMode getCurrentStrategy() const { return current_strategy_; }

    /**
     * @brief Get configured update mode
     * @return Configured UpdateMode
     */
    UpdateMode getConfiguredMode() const { return config_.update_mode; }

    /**
     * @brief Check if currently in adaptive mode
     * @return True if adaptive mode is enabled
     */
    bool isAdaptive() const { return config_.update_mode == UpdateMode::ADAPTIVE; }

    /**
     * @brief Reset adaptive state (call when starting new session)
     */
    void reset();

    /**
     * @brief Update configuration dynamically
     * @param new_config New configuration parameters
     */
    void updateConfig(const Config& new_config);

    /**
     * @brief Get current statistics for debugging
     * @return String with current state information
     */
    std::string getStatusString() const;

private:
    Config config_;                        ///< Configuration parameters
    UpdateMode current_strategy_;          ///< Currently active strategy
    int frames_below_threshold_;           ///< Counter for frames below FOV threshold
    int frames_above_threshold_;           ///< Counter for frames above FOV threshold
    mutable std::mutex mutex_;             ///< Thread safety for concurrent access

    /**
     * @brief Calculate feature density from point count and volume
     * @param point_count Number of points
     * @param volume Scan volume in m³
     * @return Feature density in points/m³
     */
    double calculateFeatureDensity(int point_count, double volume) const;

    /**
     * @brief Check if strategy should switch with hysteresis
     * @param should_use_async True if conditions favor ASYNC mode
     * @return True if strategy should change
     */
    bool shouldSwitchStrategy(bool should_use_async);
};

/**
 * @brief Utility function to convert UpdateMode to string
 * @param mode UpdateMode enum value
 * @return String representation
 */
inline const char* updateModeToString(UpdateMode mode) {
    switch (mode) {
        case UpdateMode::BUNDLE:   return "BUNDLE";
        case UpdateMode::ASYNC:    return "ASYNC";
        case UpdateMode::ADAPTIVE: return "ADAPTIVE";
        default:                   return "UNKNOWN";
    }
}

// ============================================================================
// Implementation
// ============================================================================

inline UpdateMode AdaptiveFusionManager::determineStrategy(
    int point_count,
    double scan_volume)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // If not in adaptive mode, return configured mode
    if (config_.update_mode != UpdateMode::ADAPTIVE) {
        current_strategy_ = config_.update_mode;
        return current_strategy_;
    }

    // Calculate feature density
    double density = calculateFeatureDensity(point_count, scan_volume);

    // Determine if conditions favor asynchronous processing
    bool should_use_async = (point_count >= config_.fov_threshold) &&
                           (density >= config_.feature_density_threshold);

    // Apply hysteresis logic to prevent rapid switching
    if (shouldSwitchStrategy(should_use_async)) {
        UpdateMode new_strategy = should_use_async ? UpdateMode::ASYNC : UpdateMode::BUNDLE;

        if (new_strategy != current_strategy_ && config_.enable_debug_output) {
            std::cout << "[AdaptiveFusion] Strategy switch: "
                     << updateModeToString(current_strategy_)
                     << " -> " << updateModeToString(new_strategy)
                     << " (points: " << point_count
                     << ", density: " << density << ")" << std::endl;
        }

        current_strategy_ = new_strategy;
    }

    return current_strategy_;
}

inline bool AdaptiveFusionManager::canBundle(size_t buffer_size, int num_lidars) const {
    // Need at least one scan from each LiDAR to bundle
    return buffer_size >= static_cast<size_t>(num_lidars);
}

inline void AdaptiveFusionManager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_strategy_ = UpdateMode::ASYNC;  // Start with ASYNC
    frames_below_threshold_ = 0;
    frames_above_threshold_ = 0;
}

inline void AdaptiveFusionManager::updateConfig(const Config& new_config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = new_config;
}

inline std::string AdaptiveFusionManager::getStatusString() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::ostringstream oss;
    oss << "AdaptiveFusion Status:\n"
        << "  Mode: " << updateModeToString(config_.update_mode) << "\n"
        << "  Current Strategy: " << updateModeToString(current_strategy_) << "\n"
        << "  FOV Threshold: " << config_.fov_threshold << " points\n"
        << "  Feature Density Threshold: " << config_.feature_density_threshold << " pts/m³\n"
        << "  Frames below threshold: " << frames_below_threshold_ << "\n"
        << "  Frames above threshold: " << frames_above_threshold_;

    return oss.str();
}

inline double AdaptiveFusionManager::calculateFeatureDensity(
    int point_count,
    double volume) const
{
    if (volume <= 0.0) return 0.0;
    return static_cast<double>(point_count) / volume;
}

inline bool AdaptiveFusionManager::shouldSwitchStrategy(bool should_use_async) {
    // Count frames that favor each mode
    if (should_use_async) {
        frames_above_threshold_++;
        frames_below_threshold_ = 0;
    } else {
        frames_below_threshold_++;
        frames_above_threshold_ = 0;
    }

    // Switch to ASYNC if conditions are favorable for enough frames
    if (should_use_async && current_strategy_ == UpdateMode::BUNDLE) {
        return frames_above_threshold_ >= config_.stability_frames;
    }

    // Switch to BUNDLE if conditions are unfavorable for enough frames
    if (!should_use_async && current_strategy_ == UpdateMode::ASYNC) {
        return frames_below_threshold_ >= config_.stability_frames;
    }

    return false;
}

#endif // ADAPTIVE_FUSION_HPP
