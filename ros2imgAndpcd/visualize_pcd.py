#!/usr/bin/env python3
"""Render an XYZI PCD as a top-down and three-dimensional PNG preview."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_xyzi_pcd(path: Path) -> np.ndarray:
    """Read the binary or ASCII XYZI PCD files produced by pcd_io.py."""
    with path.open("rb") as handle:
        header_lines = []
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("PCD header has no DATA line")
            decoded = line.decode("ascii").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA "):
                mode = decoded.split(maxsplit=1)[1]
                break
        header = {
            parts[0]: parts[1:]
            for line in header_lines
            if (parts := line.split()) and not line.startswith("#")
        }
        fields = header.get("FIELDS", [])
        if fields != ["x", "y", "z", "intensity"]:
            raise ValueError(f"Expected XYZI fields, got {fields}")
        point_count = int(header["POINTS"][0])
        if mode == "binary":
            points = np.frombuffer(handle.read(), dtype="<f4")
            if points.size != point_count * 4:
                raise ValueError(
                    f"PCD payload has {points.size // 4} points, expected {point_count}"
                )
            return points.reshape((-1, 4))
        if mode == "ascii":
            return np.loadtxt(handle, dtype=np.float32).reshape((-1, 4))
        raise ValueError(f"Unsupported PCD DATA mode: {mode}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-points", type=int, default=100_000)
    args = parser.parse_args()

    points = read_xyzi_pcd(args.pcd)
    finite = np.isfinite(points[:, :3]).all(axis=1)
    points = points[finite]
    if not len(points):
        raise ValueError("Point cloud contains no finite XYZ points")

    rng = np.random.default_rng(406)
    if len(points) > args.max_points:
        points = points[rng.choice(len(points), args.max_points, replace=False)]

    x, y, z, intensity = points.T
    z_low, z_high = np.percentile(z, [1, 99])
    color = np.clip(z, z_low, z_high)

    figure = plt.figure(figsize=(16, 7.5), facecolor="#101418")
    top = figure.add_subplot(1, 2, 1)
    top.set_facecolor("#101418")
    scatter = top.scatter(x, y, c=color, s=0.35, cmap="turbo", linewidths=0)
    top.scatter([0], [0], marker="^", s=80, color="white", edgecolors="black")
    top.set_aspect("equal", adjustable="box")
    top.set_title("Top view (color = height)", color="white")
    top.set_xlabel("X (m)", color="white")
    top.set_ylabel("Y (m)", color="white")
    top.tick_params(colors="white")
    top.grid(color="white", alpha=0.08)
    colorbar = figure.colorbar(scatter, ax=top, fraction=0.046, pad=0.04)
    colorbar.set_label("Z (m)", color="white")
    colorbar.ax.tick_params(colors="white")

    view = figure.add_subplot(1, 2, 2, projection="3d")
    view.set_facecolor("#101418")
    view.scatter(x, y, z, c=color, s=0.25, cmap="turbo", linewidths=0)
    view.set_title("3D perspective", color="white")
    view.set_xlabel("X (m)", color="white")
    view.set_ylabel("Y (m)", color="white")
    view.set_zlabel("Z (m)", color="white")
    view.tick_params(colors="white")
    view.view_init(elev=24, azim=-52)
    view.set_box_aspect((1, 1, 0.35))

    figure.suptitle(
        f"RoboSense point cloud — {args.pcd.name} — {len(points):,} plotted points",
        color="white",
        fontsize=14,
    )
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, facecolor=figure.get_facecolor())
    plt.close(figure)

    ranges = np.linalg.norm(points[:, :3], axis=1)
    print(f"points={len(points)}")
    print(f"x_range=[{x.min():.3f}, {x.max():.3f}]")
    print(f"y_range=[{y.min():.3f}, {y.max():.3f}]")
    print(f"z_range=[{z.min():.3f}, {z.max():.3f}]")
    print(f"range_p50={np.percentile(ranges, 50):.3f}")
    print(f"range_p95={np.percentile(ranges, 95):.3f}")
    print(f"intensity_range=[{intensity.min():.1f}, {intensity.max():.1f}]")


if __name__ == "__main__":
    main()
