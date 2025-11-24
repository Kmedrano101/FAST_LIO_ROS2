#!/usr/bin/env python3
"""
Livox to FAST-LIO Extrinsic Converter
--------------------------------------
Converts Livox driver extrinsic parameters (Euler angles + translation in mm)
to FAST-LIO format (rotation matrix + translation in meters).

Usage:
    python3 convert_livox_extrinsics.py

Author: Generated for FAST-LIO ROS2 dual MID-360 setup
Date: 2025-11-24
"""

import numpy as np
import json
from pathlib import Path


def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg, convention='ZYX'):
    """
    Convert Euler angles to rotation matrix.

    Args:
        roll_deg: Roll angle in degrees
        pitch_deg: Pitch angle in degrees
        yaw_deg: Yaw angle in degrees
        convention: Rotation convention ('ZYX' for Yaw-Pitch-Roll)

    Returns:
        3x3 rotation matrix as numpy array
    """
    # Convert to radians
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)

    # Individual rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Apply in ZYX order (Yaw-Pitch-Roll)
    if convention == 'ZYX':
        R = Rz @ Ry @ Rx
    else:
        raise ValueError(f"Unsupported convention: {convention}")

    return R


def format_rotation_matrix_yaml(R, indent=6):
    """Format rotation matrix for YAML config file."""
    indent_str = ' ' * indent
    r11, r12, r13 = R[0, :]
    r21, r22, r23 = R[1, :]
    r31, r32, r33 = R[2, :]

    return f"""[{r11:8.5f}, {r12:8.5f}, {r13:8.5f},
{indent_str} {r21:8.5f}, {r22:8.5f}, {r23:8.5f},
{indent_str} {r31:8.5f}, {r32:8.5f}, {r33:8.5f}]"""


def compute_relative_transform(T1, R1, T2, R2):
    """
    Compute relative transform: Frame2 w.r.t. Frame1

    Args:
        T1, R1: Translation (3,) and rotation (3,3) of Frame1
        T2, R2: Translation (3,) and rotation (3,3) of Frame2

    Returns:
        T_rel, R_rel: Relative translation and rotation
    """
    # T_2_wrt_1 = T2 - T1 (simplified, assumes same reference frame)
    T_rel = T2 - T1

    # R_2_wrt_1 = R2 * R1^T
    R_rel = R2 @ R1.T

    return T_rel, R_rel


def load_livox_config(config_path):
    """Load Livox driver configuration from JSON."""
    with open(config_path, 'r') as f:
        config = json.load(f)
    return config['lidar_configs']


def convert_livox_to_fastlio(livox_configs):
    """
    Convert Livox extrinsic configs to FAST-LIO format.

    Args:
        livox_configs: List of Livox lidar configurations

    Returns:
        Dictionary with FAST-LIO formatted extrinsics
    """
    results = {}

    for idx, lidar in enumerate(livox_configs):
        ip = lidar['ip']
        ext = lidar['extrinsic_parameter']

        # Extract parameters
        roll = ext['roll']
        pitch = ext['pitch']
        yaw = ext['yaw']
        x_mm = ext['x']
        y_mm = ext['y']
        z_mm = ext['z']

        # Convert translation from mm to meters
        T = np.array([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0])

        # Convert Euler angles to rotation matrix
        R = euler_to_rotation_matrix(roll, pitch, yaw)

        lidar_key = f"lidar{idx+1}" if idx > 0 else "lidar1"
        results[lidar_key] = {
            'ip': ip,
            'euler': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
            'translation_m': T,
            'rotation_matrix': R
        }

    # Compute relative transform (L2 w.r.t. L1)
    if len(results) >= 2:
        T1 = results['lidar1']['translation_m']
        R1 = results['lidar1']['rotation_matrix']
        T2 = results['lidar2']['translation_m']
        R2 = results['lidar2']['rotation_matrix']

        T_rel, R_rel = compute_relative_transform(T1, R1, T2, R2)
        results['relative'] = {
            'translation_m': T_rel,
            'rotation_matrix': R_rel
        }

    return results


def print_fastlio_config(results):
    """Print FAST-LIO YAML configuration."""
    print("\n" + "="*80)
    print("FAST-LIO EXTRINSIC CONFIGURATION")
    print("="*80)

    # LiDAR 1
    if 'lidar1' in results:
        l1 = results['lidar1']
        print(f"\n# LiDAR 1 (IP: {l1['ip']}) → Base Link")
        print(f"# Euler: Roll={l1['euler']['roll']}°, Pitch={l1['euler']['pitch']}°, Yaw={l1['euler']['yaw']}°")
        print(f"extrinsic_T: [{l1['translation_m'][0]:.2f}, {l1['translation_m'][1]:.2f}, {l1['translation_m'][2]:.2f}]")
        print(f"extrinsic_R: {format_rotation_matrix_yaml(l1['rotation_matrix'], indent=13)}")

    # LiDAR 2
    if 'lidar2' in results:
        l2 = results['lidar2']
        print(f"\n# LiDAR 2 (IP: {l2['ip']}) → Base Link")
        print(f"# Euler: Roll={l2['euler']['roll']}°, Pitch={l2['euler']['pitch']}°, Yaw={l2['euler']['yaw']}°")
        print(f"extrinsic_T2: [{l2['translation_m'][0]:.2f}, {l2['translation_m'][1]:.2f}, {l2['translation_m'][2]:.2f}]")
        print(f"extrinsic_R2: {format_rotation_matrix_yaml(l2['rotation_matrix'], indent=14)}")

    # Relative transform
    if 'relative' in results:
        rel = results['relative']
        baseline = np.linalg.norm(rel['translation_m'])
        print(f"\n# Relative Transform: LiDAR 2 w.r.t. LiDAR 1")
        print(f"# Baseline distance: {baseline*1000:.1f}mm")
        print(f"extrinsic_T_L2_wrt_L1: [{rel['translation_m'][0]:.2f}, {rel['translation_m'][1]:.2f}, {rel['translation_m'][2]:.2f}]")
        print(f"extrinsic_R_L2_wrt_L1: {format_rotation_matrix_yaml(rel['rotation_matrix'], indent=23)}")

    print("\n" + "="*80)


def print_verification_info(results):
    """Print verification and analysis information."""
    print("\n" + "="*80)
    print("VERIFICATION & ANALYSIS")
    print("="*80)

    if 'lidar1' in results and 'lidar2' in results:
        l1 = results['lidar1']
        l2 = results['lidar2']

        # Analyze orientations
        print("\n## Sensor Orientations:")
        print(f"  LiDAR 1 ({l1['ip']}): ", end="")
        if abs(l1['euler']['yaw'] - 0) < 1:
            print("Facing FORWARD (0°)")
        elif abs(l1['euler']['yaw'] - 180) < 1 or abs(l1['euler']['yaw'] + 180) < 1:
            print("Facing BACKWARD (180°)")
        elif abs(l1['euler']['yaw'] - 90) < 1:
            print("Facing LEFT (90°)")
        elif abs(l1['euler']['yaw'] + 90) < 1:
            print("Facing RIGHT (-90°)")
        else:
            print(f"Custom angle ({l1['euler']['yaw']}°)")

        print(f"  LiDAR 2 ({l2['ip']}): ", end="")
        if abs(l2['euler']['yaw'] - 0) < 1:
            print("Facing FORWARD (0°)")
        elif abs(l2['euler']['yaw'] - 180) < 1 or abs(l2['euler']['yaw'] + 180) < 1:
            print("Facing BACKWARD (180°)")
        elif abs(l2['euler']['yaw'] - 90) < 1:
            print("Facing LEFT (90°)")
        elif abs(l2['euler']['yaw'] + 90) < 1:
            print("Facing RIGHT (-90°)")
        else:
            print(f"Custom angle ({l2['euler']['yaw']}°)")

        # Baseline analysis
        if 'relative' in results:
            T_rel = results['relative']['translation_m']
            baseline = np.linalg.norm(T_rel)
            print(f"\n## Baseline Distance: {baseline*1000:.1f}mm")
            print(f"  Lateral separation: {abs(T_rel[1])*1000:.1f}mm")
            print(f"  Longitudinal offset: {abs(T_rel[0])*1000:.1f}mm")
            print(f"  Vertical offset: {abs(T_rel[2])*1000:.1f}mm")

            # Check if reasonable
            if baseline < 0.05:
                print("  ⚠️  WARNING: Baseline very small (<5cm), may not provide good stereo effect")
            elif baseline > 1.0:
                print("  ⚠️  WARNING: Baseline very large (>1m), verify measurements")
            else:
                print("  ✓ Baseline looks reasonable")

        # Rotation matrix determinant check
        print("\n## Rotation Matrix Validation:")
        det1 = np.linalg.det(l1['rotation_matrix'])
        det2 = np.linalg.det(l2['rotation_matrix'])
        print(f"  LiDAR 1 det(R) = {det1:.6f} (should be ≈ 1.0)")
        print(f"  LiDAR 2 det(R) = {det2:.6f} (should be ≈ 1.0)")

        if abs(det1 - 1.0) > 0.01 or abs(det2 - 1.0) > 0.01:
            print("  ⚠️  WARNING: Determinant not close to 1.0, rotation may be invalid")
        else:
            print("  ✓ Rotation matrices are valid")

        # Orthogonality check
        I1 = l1['rotation_matrix'] @ l1['rotation_matrix'].T
        I2 = l2['rotation_matrix'] @ l2['rotation_matrix'].T
        ortho_err1 = np.max(np.abs(I1 - np.eye(3)))
        ortho_err2 = np.max(np.abs(I2 - np.eye(3)))
        print(f"  LiDAR 1 orthogonality error: {ortho_err1:.6f}")
        print(f"  LiDAR 2 orthogonality error: {ortho_err2:.6f}")

        if ortho_err1 > 0.01 or ortho_err2 > 0.01:
            print("  ⚠️  WARNING: Matrices not orthogonal, may have numerical errors")
        else:
            print("  ✓ Rotation matrices are orthogonal")

    print("\n" + "="*80)


def main():
    """Main conversion script."""
    print("Livox to FAST-LIO Extrinsic Converter")
    print("="*80)

    # Default config path
    config_path = Path("/home/jetson/ros2_ws/src/livox_ros_driver2/config/multiple_netconfigs.json")

    # Check if file exists
    if not config_path.exists():
        print(f"❌ Error: Config file not found at {config_path}")
        print("\nPlease provide the correct path to multiple_netconfigs.json")
        return

    print(f"📁 Reading Livox config from: {config_path}")

    # Load and convert
    livox_configs = load_livox_config(config_path)
    print(f"✓ Found {len(livox_configs)} LiDAR configurations")

    results = convert_livox_to_fastlio(livox_configs)

    # Print FAST-LIO config
    print_fastlio_config(results)

    # Print verification info
    print_verification_info(results)

    # Usage instructions
    print("\n" + "="*80)
    print("NEXT STEPS")
    print("="*80)
    print("1. Copy the extrinsic parameters above")
    print("2. Paste into: /home/jetson/ros2_ws/src/fast_lio_ros2/config/dual_mid360_mine.yaml")
    print("3. Update the 'mapping:' section with the extrinsic_T, extrinsic_R, etc.")
    print("4. Verify sensor alignment in RViz2 after launch")
    print("5. Check documentation: fast_lio_ros2/docs/EXTRINSIC_CALIBRATION_DUAL_LIDAR.md")
    print("="*80 + "\n")


if __name__ == "__main__":
    main()
