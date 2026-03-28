"""IMU time-series visualization with covariance bands.

Reads a CSV exported from ``sensor_msgs/Imu`` and plots orientation,
angular velocity, and linear acceleration against time with 1-sigma
covariance shading for the corresponding axes.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SIGMA_SCALE = 1.0


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_csv = script_dir / "data" / "imu-data.csv"

    parser = argparse.ArgumentParser(
        description="Plot IMU data and covariance over time."
    )
    parser.add_argument(
        "csv_path",
        nargs="?",
        default=default_csv,
        help="Path to the IMU CSV file.",
    )
    parser.add_argument(
        "--cov-start",
        type=float,
        default=5.0,
        help="Start time in seconds for the covariance estimation window.",
    )
    parser.add_argument(
        "--cov-end",
        type=float,
        default=15.0,
        help="End time in seconds for the covariance estimation window.",
    )
    return parser.parse_args()


def quaternion_to_euler_xyz(x: np.ndarray, y: np.ndarray, z: np.ndarray, w: np.ndarray):
    """Convert quaternions to roll, pitch, yaw in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def load_imu_csv(csv_path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    df = pd.read_csv(csv_path)

    time = df["Time"].to_numpy(dtype=float)
    time = time - time[0]

    quaternion = df[
        ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
    ].to_numpy(dtype=float)

    roll, pitch, yaw = quaternion_to_euler_xyz(
        quaternion[:, 0],
        quaternion[:, 1],
        quaternion[:, 2],
        quaternion[:, 3],
    )

    signals = {
        "quaternion": quaternion,
        "orientation_rpy_rad": np.column_stack([roll, pitch, yaw]),
        "orientation": np.rad2deg(np.column_stack([roll, pitch, yaw])),
        "angular_velocity": df[
            ["angular_velocity.x", "angular_velocity.y", "angular_velocity.z"]
        ].to_numpy(dtype=float),
        "linear_acceleration": df[
            ["linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"]
        ].to_numpy(dtype=float),
    }

    return time, signals



def compute_cov(data):
    x = np.asarray(data, dtype=float)
    return np.cov(x, rowvar=False, ddof=1)

def select_covariance_window(
    time: np.ndarray,
    cov_start: float,
    cov_end: float,
) -> tuple[slice, str]:
    if cov_end <= cov_start:
        raise ValueError("cov-end must be greater than cov-start")

    start_idx = int(np.searchsorted(time, cov_start, side="left"))
    end_idx = int(np.searchsorted(time, cov_end, side="right"))

    if end_idx - start_idx < 2:
        raise ValueError(
            f"Covariance window {cov_start:.2f}s to {cov_end:.2f}s does not contain enough samples"
        )

    return slice(start_idx, end_idx), f"{cov_start:.2f}s to {cov_end:.2f}s ({end_idx - start_idx} samples)"


def compute_window_covariances(
    signals: dict[str, np.ndarray],
    window: slice,
) -> dict[str, np.ndarray]:
    # Remove 2*pi discontinuities before covariance calculation.
    orientation_window = np.unwrap(signals["orientation_rpy_rad"][window], axis=0)

    covariance_samples = {
        "orientation": orientation_window,
        "angular_velocity": signals["angular_velocity"][window],
        "linear_acceleration": signals["linear_acceleration"][window],
    }

    return {name: compute_cov(data) for name, data in covariance_samples.items()}


def print_covariances(covariances: dict[str, np.ndarray], window_desc: str):
    print(f"\nEstimated covariances from CSV data window ({window_desc}):")
    np.set_printoptions(precision=8, suppress=False)

    print("\nOrientation covariance (roll, pitch, yaw) [rad^2]:")
    print(covariances["orientation"])

    print("\nAngular velocity covariance (x, y, z) [rad^2/s^2]:")
    print(covariances["angular_velocity"])

    print("\nLinear acceleration covariance (x, y, z) [(m/s^2)^2]:")
    print(covariances["linear_acceleration"])


def covariance_diagonals_for_plot(
    time: np.ndarray, covariances: dict[str, np.ndarray]
) -> dict[str, np.ndarray]:
    orientation_diag_deg2 = np.rad2deg(np.sqrt(np.clip(np.diag(covariances["orientation"]), 0.0, None))) ** 2

    return {
        "orientation": np.tile(orientation_diag_deg2, (len(time), 1)),
        "angular_velocity": np.tile(np.diag(covariances["angular_velocity"]), (len(time), 1)),
        "linear_acceleration": np.tile(np.diag(covariances["linear_acceleration"]), (len(time), 1)),
    }


def plot_series_with_covariance(
    ax: plt.Axes,
    time: np.ndarray,
    values: np.ndarray,
    variances: np.ndarray,
    labels: list[str],
    colors: list[str],
    title: str,
    ylabel: str,
):
    for axis_idx, (label, color) in enumerate(zip(labels, colors)):
        series = values[:, axis_idx]
        sigma = SIGMA_SCALE * np.sqrt(np.clip(variances[:, axis_idx], 0.0, None))

        ax.plot(time, series, color=color, linewidth=1.4, label=label)
        ax.fill_between(
            time,
            series - sigma,
            series + sigma,
            color=color,
            alpha=0.18,
            linewidth=0.0,
        )

    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")


def visualize_imu_data(time: np.ndarray, signals: dict[str, np.ndarray], covariances: dict[str, np.ndarray]):
    plt.style.use("default")

    fig, axes = plt.subplots(4, 1, figsize=(15, 13), sharex=True, facecolor="#f7f4ea")
    fig.suptitle("IMU Signal Analysis", fontsize=15, fontweight="bold", color="#1f2933")

    axis_colors = ["#58a6ff", "#3fb950", "#f778ba"]
    axis_labels = ["x", "y", "z"]
    quaternion_colors = ["#58a6ff", "#3fb950", "#f778ba", "#d29922"]
    quaternion_labels = ["qx", "qy", "qz", "qw"]

    for ax in axes:
        ax.set_facecolor("#fffdf7")
        ax.tick_params(colors="#1f2933")
        for spine in ax.spines.values():
            spine.set_color("#52606d")

    plot_series_with_covariance(
        axes[0],
        time,
        signals["quaternion"],
        np.zeros_like(signals["quaternion"]),
        quaternion_labels,
        quaternion_colors,
        "Orientation Quaternion vs Time",
        "Quaternion",
    )

    plot_series_with_covariance(
        axes[1],
        time,
        signals["orientation"],
        covariances["orientation"],
        ["roll", "pitch", "yaw"],
        axis_colors,
        "Orientation vs Time",
        "Angle (deg)",
    )

    plot_series_with_covariance(
        axes[2],
        time,
        signals["angular_velocity"],
        covariances["angular_velocity"],
        axis_labels,
        axis_colors,
        "Angular Velocity vs Time",
        "Angular Velocity (rad/s)",
    )

    plot_series_with_covariance(
        axes[3],
        time,
        signals["linear_acceleration"],
        covariances["linear_acceleration"],
        axis_labels,
        axis_colors,
        "Linear Acceleration vs Time",
        "Acceleration (m/s^2)",
    )

    axes[3].set_xlabel("Time Since Start (s)")
    axes[3].xaxis.label.set_color("#1f2933")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    plt.show()


def main():
    args = parse_args()
    csv_path = Path(args.csv_path).expanduser().resolve()
    time, signals = load_imu_csv(csv_path)

    covariance_window, window_desc = select_covariance_window(
        time,
        args.cov_start,
        args.cov_end,
    )
    covariances = compute_window_covariances(signals, covariance_window)
    print_covariances(covariances, window_desc)

    visualize_imu_data(time, signals, covariance_diagonals_for_plot(time, covariances))


if __name__ == "__main__":
    main()
