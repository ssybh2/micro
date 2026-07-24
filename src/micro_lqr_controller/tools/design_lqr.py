#!/usr/bin/env python3
"""Reproduce the ROS 2 node's exact-ZOH discrete LQR design."""
from __future__ import annotations

import argparse
import numpy as np
from scipy.linalg import expm, solve_discrete_are, eigvals


def design(
    body_mass: float = 2.54,
    non_pitch_mass: float = 0.26,
    wheel_inertia_each: float = 0.0,
    com_height: float = 0.120,
    body_pitch_inertia: float = 0.036576,
    wheel_radius: float = 0.030,
    gravity: float = 9.80665,
    dt: float = 0.003,
    q=(1.0, 1.0, 1.0, 1.0),
    r_cost: float = 10.0,
    course_legacy_b4: bool = False,
):
    m = body_mass
    M = non_pitch_mass + 2.0 * wheel_inertia_each / wheel_radius**2
    h = com_height
    I = body_pitch_inertia
    den = (M + m) * (m * h**2 + I) - m**2 * h**2
    if den <= 0:
        raise ValueError("Model denominator is non-positive")

    A = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [((M + m) * m * gravity * h) / den, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [-(m**2 * gravity * h**2) / den, 0.0, 0.0, 0.0],
        ]
    )
    if course_legacy_b4:
        b4 = 1.0 / ((M + m) * wheel_radius) - (
            m**2 * h**2 / ((M + m) * den * wheel_radius)
        )
    else:
        b4 = (m * h**2 + I) / (den * wheel_radius)
    B = np.array([[0.0], [-(m * h) / (den * wheel_radius)], [0.0], [b4]])

    augmented = np.zeros((5, 5))
    augmented[:4, :4] = A
    augmented[:4, 4:] = B
    exact = expm(augmented * dt)
    Ad = exact[:4, :4]
    Bd = exact[:4, 4:]
    Q = np.diag(q)
    R = np.array([[r_cost]])
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(Bd.T @ P @ Bd + R, Bd.T @ P @ Ad)
    poles = eigvals(Ad - Bd @ K)
    ctrb = np.column_stack([Bd, Ad @ Bd, Ad @ Ad @ Bd, Ad @ Ad @ Ad @ Bd])
    return A, B, Ad, Bd, K, poles, np.linalg.matrix_rank(ctrb)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--course-legacy-b4", action="store_true")
    parser.add_argument("--r", type=float, default=10.0)
    args = parser.parse_args()
    _, _, _, _, K, poles, rank = design(
        r_cost=args.r, course_legacy_b4=args.course_legacy_b4
    )
    print("state order: [pitch, pitch_rate, x, x_dot]")
    print("model input: total axle torque [N*m]")
    print("K:", np.array2string(K, precision=12))
    print("closed-loop poles:", np.array2string(poles, precision=12))
    print("max pole magnitude:", float(np.max(np.abs(poles))))
    print("controllability rank:", rank)


if __name__ == "__main__":
    main()
