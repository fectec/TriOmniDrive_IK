#!/usr/bin/env python3

import numpy as np

def build_kinematic_matrix(alphas_deg):
    """
    Build the 3x3 forward kinematics matrix M for a 3-wheel omnidirectional robot,
    given the wheel orientations in degrees.
    """
    # Convert degrees to radians
    alphas_rad = np.deg2rad(alphas_deg)
    # Build matrix M
    M = np.array([
        [np.cos(alphas_rad[0] + np.pi/2),
         np.cos(alphas_rad[1] + np.pi/2),
         np.cos(alphas_rad[2] + np.pi/2)],
        [np.sin(alphas_rad[0] + np.pi/2),
         np.sin(alphas_rad[1] + np.pi/2),
         np.sin(alphas_rad[2] + np.pi/2)],
        [1.0, 1.0, 1.0]
    ])
    return M

def compute_wheel_angular_velocities(s, wheel_radius):
    """
    Convert linear wheel speeds (m/s) to angular wheel speeds (rad/s).
    """
    return s / wheel_radius

def main():
    # 1) Define wheel angles (degrees)
    alpha_1, alpha_2, alpha_3 = 30, 150, 270

    # 2) Build M matrix
    M = build_kinematic_matrix([alpha_1, alpha_2, alpha_3])
    print("M (Forward Kinematics):")
    print(M)

    # 3) Compute inverse of M
    Minv = np.linalg.inv(M)
    print("\nM⁻¹ (Inverse Kinematics):")
    print(Minv)

    # 4) Example desired velocity v = [vx, vy, ω]
    vx, vy, omega = 0.0, 1.0, 0.0  # m/s, m/s, rad/s
    v = np.array([vx, vy, omega])

    # 5) Compute linear wheel speeds s = [s1, s2, s3] (m/s)
    s = Minv @ v
    print(f"\nExample usage:\n  v = [{vx}, {vy}, {omega}] → [s1, s2, s3] = {s} (m/s)")

    # 6) Convert to wheel angular velocities (rad/s)
    wheel_radius = 0.0240   # meters
    w = compute_wheel_angular_velocities(s, wheel_radius)
    print(f"\nWheel angular velocities (rad/s) for wheel radius {wheel_radius} m:")
    print(f"  [ω1, ω2, ω3] = {w}")

if __name__ == "__main__":
    main()