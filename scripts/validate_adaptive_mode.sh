#!/bin/bash
################################################################################
# Adaptive Mode Validation Script for Dual MID-360 Setup
#
# This script validates that the adaptive fusion mode is properly implemented
# and configured for the dual Livox MID-360 LiDAR system.
#
# Usage: ./validate_adaptive_mode.sh
################################################################################

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PKG_ROOT="$WS_ROOT/src/fast_lio_ros2"

echo "=================================="
echo "Adaptive Mode Validation Script"
echo "=================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0

# Function to check and report
check_pass() {
    echo -e "${GREEN}✓ PASS:${NC} $1"
    ((PASS_COUNT++))
}

check_fail() {
    echo -e "${RED}✗ FAIL:${NC} $1"
    ((FAIL_COUNT++))
}

check_warn() {
    echo -e "${YELLOW}⚠ WARN:${NC} $1"
}

echo "1. Checking File Structure..."
echo "=================================="

# Check adaptive_fusion.hpp exists
if [ -f "$PKG_ROOT/src/adaptive_fusion.hpp" ]; then
    check_pass "adaptive_fusion.hpp exists"
else
    check_fail "adaptive_fusion.hpp not found"
fi

# Check laserMapping.cpp exists
if [ -f "$PKG_ROOT/src/laserMapping.cpp" ]; then
    check_pass "laserMapping.cpp exists"
else
    check_fail "laserMapping.cpp not found"
fi

# Check configuration file exists
if [ -f "$PKG_ROOT/config/dual_mid360_mine.yaml" ]; then
    check_pass "Configuration file exists"
else
    check_fail "Configuration file not found"
fi

# Check documentation exists
if [ -f "$PKG_ROOT/docs/ADAPTIVE_MODE_GUIDE.md" ]; then
    check_pass "Adaptive mode guide exists"
else
    check_warn "Adaptive mode guide not found (optional)"
fi

echo ""
echo "2. Checking Implementation..."
echo "=================================="

# Check if adaptive_fusion.hpp is included in laserMapping.cpp
if grep -q '#include.*adaptive_fusion.hpp' "$PKG_ROOT/src/laserMapping.cpp"; then
    check_pass "adaptive_fusion.hpp is included in laserMapping.cpp"
else
    check_fail "adaptive_fusion.hpp not included in laserMapping.cpp"
fi

# Check UpdateMode enum exists
if grep -q 'enum class UpdateMode' "$PKG_ROOT/src/adaptive_fusion.hpp"; then
    check_pass "UpdateMode enum defined"
else
    check_fail "UpdateMode enum not found"
fi

# Check AdaptiveFusionManager class exists
if grep -q 'class AdaptiveFusionManager' "$PKG_ROOT/src/adaptive_fusion.hpp"; then
    check_pass "AdaptiveFusionManager class defined"
else
    check_fail "AdaptiveFusionManager class not found"
fi

# Check determineStrategy method exists
if grep -q 'determineStrategy' "$PKG_ROOT/src/adaptive_fusion.hpp"; then
    check_pass "determineStrategy() method defined"
else
    check_fail "determineStrategy() method not found"
fi

# Check adaptive_fusion_manager instantiation
if grep -q 'adaptive_fusion_manager.*make_unique\|adaptive_fusion_manager.*new' "$PKG_ROOT/src/laserMapping.cpp"; then
    check_pass "AdaptiveFusionManager instantiated"
else
    check_fail "AdaptiveFusionManager not instantiated"
fi

# Check strategy execution logic
if grep -q 'UpdateMode.*BUNDLE\|sync_packages_bundle' "$PKG_ROOT/src/laserMapping.cpp"; then
    check_pass "Bundle mode logic implemented"
else
    check_fail "Bundle mode logic not found"
fi

# Check async mode fallback
if grep -q 'sync_packages.*Measures' "$PKG_ROOT/src/laserMapping.cpp"; then
    check_pass "Async mode logic implemented"
else
    check_fail "Async mode logic not found"
fi

echo ""
echo "3. Checking Configuration..."
echo "=================================="

# Check update_mode parameter
if grep -q 'update_mode:.*2' "$PKG_ROOT/config/dual_mid360_mine.yaml"; then
    check_pass "update_mode set to 2 (Adaptive)"
else
    check_warn "update_mode not set to 2 (may be Bundle or Async)"
fi

# Check adaptive parameters
if grep -q 'adaptive_fov_threshold:' "$PKG_ROOT/config/dual_mid360_mine.yaml"; then
    check_pass "adaptive_fov_threshold configured"
else
    check_fail "adaptive_fov_threshold not configured"
fi

if grep -q 'adaptive_feature_density:' "$PKG_ROOT/config/dual_mid360_mine.yaml"; then
    check_pass "adaptive_feature_density configured"
else
    check_fail "adaptive_feature_density not configured"
fi

if grep -q 'adaptive_hysteresis_ratio:' "$PKG_ROOT/config/dual_mid360_mine.yaml"; then
    check_pass "adaptive_hysteresis_ratio configured"
else
    check_fail "adaptive_hysteresis_ratio not configured"
fi

if grep -q 'adaptive_stability_frames:' "$PKG_ROOT/config/dual_mid360_mine.yaml"; then
    check_pass "adaptive_stability_frames configured"
else
    check_fail "adaptive_stability_frames not configured"
fi

# Check multi_lidar enabled
if grep -q 'multi_lidar:.*true' "$PKG_ROOT/config/dual_mid360_mine.yaml"; then
    check_pass "multi_lidar enabled"
else
    check_fail "multi_lidar not enabled (required for adaptive mode)"
fi

echo ""
echo "4. Checking Build Status..."
echo "=================================="

# Check if package is built
if [ -f "$WS_ROOT/install/fast_lio_ros2/lib/fast_lio_ros2/fastlio_mapping" ]; then
    check_pass "Package built successfully"
else
    check_fail "Package not built (run: colcon build --packages-select fast_lio_ros2)"
fi

echo ""
echo "5. Checking Launch Files..."
echo "=================================="

# Check main launch file
if [ -f "$PKG_ROOT/launch/mapping.launch.py" ]; then
    check_pass "Main launch file exists"
else
    check_fail "Main launch file not found"
fi

# Check test launch files
TEST_LAUNCHES=("test_1_raw_lidar.launch.py" "test_2_extrinsic_validation.launch.py"
               "test_3_single_lidar.launch.py" "test_4_dual_static.launch.py")

for launch in "${TEST_LAUNCHES[@]}"; do
    if [ -f "$PKG_ROOT/launch/$launch" ]; then
        check_pass "Test launch: $launch"
    else
        check_warn "Test launch not found: $launch"
    fi
done

echo ""
echo "=================================="
echo "VALIDATION SUMMARY"
echo "=================================="
echo -e "${GREEN}Passed: $PASS_COUNT${NC}"
echo -e "${RED}Failed: $FAIL_COUNT${NC}"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${GREEN}✓ ALL CHECKS PASSED!${NC}"
    echo ""
    echo "Adaptive mode implementation is complete and ready for testing."
    echo ""
    echo "Next Steps:"
    echo "1. Source workspace: source $WS_ROOT/install/setup.bash"
    echo "2. Run tests following: $PKG_ROOT/docs/QUICK_TEST_GUIDE.md"
    echo "3. Start with Test 1-4, then proceed to Test 5C (Adaptive mode)"
    echo ""
    exit 0
else
    echo -e "${RED}✗ VALIDATION FAILED${NC}"
    echo ""
    echo "Please fix the failed checks above before proceeding."
    echo ""
    exit 1
fi
