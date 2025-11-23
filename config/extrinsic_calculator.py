#!/usr/bin/env python3
"""
Extrinsic Calibration Helper Tool
==================================
Calculate rotation matrices from Euler angles for FAST-LIO config files

Usage:
    python3 extrinsic_calculator.py

Author: FAST-LIO Multi-Sensor Team
Date: 2025-11-22
"""

import numpy as np
import sys


def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """
    Convert Euler angles (degrees) to 3x3 rotation matrix

    Args:
        roll_deg (float): Roll angle in degrees (rotation around X-axis)
        pitch_deg (float): Pitch angle in degrees (rotation around Y-axis)
        yaw_deg (float): Yaw angle in degrees (rotation around Z-axis)

    Returns:
        numpy.ndarray: 3x3 rotation matrix

    Rotation order: ZYX (yaw → pitch → roll)
    This is the standard convention in robotics (ROS REP-103)
    """
    # Convert degrees to radians
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)

    # Individual rotation matrices
    # Roll (rotation around X-axis)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Pitch (rotation around Y-axis)
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Yaw (rotation around Z-axis)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix (ZYX order)
    R = Rz @ Ry @ Rx

    return R


def format_for_yaml(R, indent=0):
    """
    Format rotation matrix for YAML configuration file

    Args:
        R (numpy.ndarray): 3x3 rotation matrix
        indent (int): Number of spaces to indent

    Returns:
        str: Formatted string for YAML
    """
    spaces = " " * indent
    output = f"{spaces}[{R[0,0]:8.5f}, {R[0,1]:8.5f}, {R[0,2]:8.5f},\n"
    output += f"{spaces} {R[1,0]:8.5f}, {R[1,1]:8.5f}, {R[1,2]:8.5f},\n"
    output += f"{spaces} {R[2,0]:8.5f}, {R[2,1]:8.5f}, {R[2,2]:8.5f}]"
    return output


def rotation_matrix_to_euler(R):
    """
    Convert rotation matrix back to Euler angles (for verification)

    Args:
        R (numpy.ndarray): 3x3 rotation matrix

    Returns:
        tuple: (roll_deg, pitch_deg, yaw_deg) in degrees
    """
    # Extract angles (ZYX convention)
    pitch = np.arcsin(-R[2, 0])

    # Check for gimbal lock
    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock case
        roll = 0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    # Convert to degrees
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def print_common_configurations():
    """Print common sensor orientations"""
    print("\n" + "="*70)
    print("COMMON SENSOR ORIENTATIONS")
    print("="*70)

    configs = [
        ("Forward (aligned with base)", 0, 0, 0),
        ("Backward (rear-facing)", 0, 0, 180),
        ("Left-facing", 0, 0, 90),
        ("Right-facing", 0, 0, -90),
        ("Upward (ceiling)", 90, 0, 0),
        ("Downward (floor)", -90, 0, 0),
        ("Rear + Tilt up 15°", 0, 15, 180),
        ("Rear + Tilt down 15°", 0, -15, 180),
    ]

    for name, roll, pitch, yaw in configs:
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        print(f"\n{name}:")
        print(f"  Euler: Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")
        print(f"  Matrix:\n{format_for_yaml(R, indent=4)}")


def interactive_calculator():
    """Interactive mode for calculating custom rotations"""
    print("\n" + "="*70)
    print("CUSTOM ROTATION CALCULATOR")
    print("="*70)
    print("\nEnter Euler angles in degrees (or 'q' to quit):")

    while True:
        try:
            # Get roll angle
            roll_input = input("\n  Roll (rotation around X-axis, °): ").strip()
            if roll_input.lower() == 'q':
                break
            roll = float(roll_input)

            # Get pitch angle
            pitch_input = input("  Pitch (rotation around Y-axis, °): ").strip()
            if pitch_input.lower() == 'q':
                break
            pitch = float(pitch_input)

            # Get yaw angle
            yaw_input = input("  Yaw (rotation around Z-axis, °): ").strip()
            if yaw_input.lower() == 'q':
                break
            yaw = float(yaw_input)

            # Calculate rotation matrix
            R = euler_to_rotation_matrix(roll, pitch, yaw)

            # Verify by converting back
            roll_check, pitch_check, yaw_check = rotation_matrix_to_euler(R)

            # Display results
            print("\n" + "-"*70)
            print("RESULT:")
            print("-"*70)
            print(f"\nInput Euler Angles:")
            print(f"  Roll:  {roll:7.2f}°")
            print(f"  Pitch: {pitch:7.2f}°")
            print(f"  Yaw:   {yaw:7.2f}°")

            print(f"\nRotation Matrix:")
            print(format_for_yaml(R, indent=2))

            print(f"\nVerification (back to Euler):")
            print(f"  Roll:  {roll_check:7.2f}°")
            print(f"  Pitch: {pitch_check:7.2f}°")
            print(f"  Yaw:   {yaw_check:7.2f}°")

            print(f"\nFor YAML config file, copy:")
            print(f"\n  extrinsic_R: {format_for_yaml(R, indent=0)}")

            print("\n" + "-"*70)

        except ValueError:
            print("  Error: Please enter valid numbers")
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break


def validate_extrinsics(T1, R1, T2, R2):
    """
    Validate extrinsic parameters

    Args:
        T1: Translation for LiDAR 1 [x, y, z]
        R1: Rotation matrix for LiDAR 1 (3x3)
        T2: Translation for LiDAR 2 [x, y, z]
        R2: Rotation matrix for LiDAR 2 (3x3)

    Returns:
        bool: True if valid, False otherwise
    """
    print("\n" + "="*70)
    print("EXTRINSIC VALIDATION")
    print("="*70)

    # Check rotation matrix validity
    def is_valid_rotation(R, name):
        # Check determinant (should be 1)
        det = np.linalg.det(R)
        if not np.isclose(det, 1.0, atol=0.01):
            print(f"❌ {name}: Invalid determinant = {det:.4f} (should be 1.0)")
            return False

        # Check orthogonality (R^T * R should be identity)
        I = np.eye(3)
        RTR = R.T @ R
        if not np.allclose(RTR, I, atol=0.01):
            print(f"❌ {name}: Not orthogonal (R^T * R != I)")
            return False

        print(f"✓ {name}: Valid rotation matrix")
        return True

    # Check translations
    def check_translation(T, name):
        if np.linalg.norm(T) > 2.0:  # More than 2 meters
            print(f"⚠️  {name}: Large offset = {np.linalg.norm(T):.2f}m (verify measurement)")
        else:
            print(f"✓ {name}: Translation = {np.linalg.norm(T):.2f}m")

    # Validate
    valid = True
    valid &= is_valid_rotation(R1, "LiDAR 1 rotation")
    valid &= is_valid_rotation(R2, "LiDAR 2 rotation")
    check_translation(T1, "LiDAR 1 translation")
    check_translation(T2, "LiDAR 2 translation")

    # Compute relative transform
    T_rel = T2 - T1
    baseline = np.linalg.norm(T_rel)
    print(f"\nRelative baseline between sensors: {baseline:.3f}m")

    if baseline < 0.05:
        print(f"⚠️  WARNING: Sensors very close (<5cm). Consider larger separation for better coverage.")
    elif baseline > 1.0:
        print(f"⚠️  WARNING: Sensors far apart (>{1.0}m). Verify measurements.")
    else:
        print(f"✓ Good sensor separation")

    return valid


def main():
    """Main function"""
    print("\n" + "="*70)
    print("FAST-LIO EXTRINSIC CALIBRATION HELPER")
    print("="*70)
    print("\nThis tool helps you:")
    print("  1. Convert Euler angles → Rotation matrices")
    print("  2. View common sensor orientations")
    print("  3. Validate extrinsic parameters")

    while True:
        print("\n" + "-"*70)
        print("OPTIONS:")
        print("  [1] Show common sensor orientations")
        print("  [2] Calculate custom rotation matrix")
        print("  [3] Validate extrinsic parameters")
        print("  [q] Quit")
        print("-"*70)

        choice = input("\nSelect option: ").strip().lower()

        if choice == '1':
            print_common_configurations()

        elif choice == '2':
            interactive_calculator()

        elif choice == '3':
            print("\nEnter extrinsic parameters:")
            try:
                print("\nLiDAR 1 Translation [x, y, z] in meters:")
                T1 = np.array([
                    float(input("  x: ")),
                    float(input("  y: ")),
                    float(input("  z: "))
                ])

                print("\nLiDAR 1 Rotation (Euler angles in degrees):")
                roll1 = float(input("  Roll: "))
                pitch1 = float(input("  Pitch: "))
                yaw1 = float(input("  Yaw: "))
                R1 = euler_to_rotation_matrix(roll1, pitch1, yaw1)

                print("\nLiDAR 2 Translation [x, y, z] in meters:")
                T2 = np.array([
                    float(input("  x: ")),
                    float(input("  y: ")),
                    float(input("  z: "))
                ])

                print("\nLiDAR 2 Rotation (Euler angles in degrees):")
                roll2 = float(input("  Roll: "))
                pitch2 = float(input("  Pitch: "))
                yaw2 = float(input("  Yaw: "))
                R2 = euler_to_rotation_matrix(roll2, pitch2, yaw2)

                validate_extrinsics(T1, R1, T2, R2)

            except ValueError:
                print("Error: Invalid input")

        elif choice == 'q':
            print("\nExiting... Good luck with your calibration!\n")
            break

        else:
            print("Invalid choice. Please select 1, 2, 3, or q")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting...\n")
        sys.exit(0)
