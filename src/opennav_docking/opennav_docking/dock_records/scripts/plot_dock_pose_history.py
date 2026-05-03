#!/usr/bin/env python3
# pyright: reportMissingImports=false

import argparse
import csv
import math
import statistics
from pathlib import Path

import matplotlib.pyplot as plt


DEFAULT_RECORD_DIR = Path(__file__).resolve().parents[1]
MAP_WIDTH_M = 3.0
MAP_HEIGHT_M = 2.2


def resolve_csv_path(csv_or_dir: Path | None) -> Path:
    target = csv_or_dir if csv_or_dir is not None else DEFAULT_RECORD_DIR

    if target.is_file():
        return target

    if not target.exists():
        raise FileNotFoundError(f"Directory not found: {target}")

    if not target.is_dir():
        raise ValueError(f"Not a file or directory: {target}")

    candidates = sorted(
        target.glob("dph*.csv"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(
            f"No dock_pose_history*.csv found in directory: {target}"
        )

    return candidates[0]


def load_csv(csv_path: Path):
    t, x, y, z, yaw = [], [], [], [], []
    frame_id = None

    with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
        sample = f.read(4096)
        f.seek(0)

        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
        except csv.Error:
            dialect = csv.excel

        reader = csv.DictReader(f, dialect=dialect)

        if reader.fieldnames is not None:
            normalized = {name.strip(): name for name in reader.fieldnames if name is not None}

            def pick(*candidates: str) -> str | None:
                for c in candidates:
                    if c in normalized:
                        return normalized[c]
                return None

            time_ns_key = pick("time_ns", "stamp_ns")
            time_key = pick("time_sec", "time", "stamp_sec", "t")
            frame_key = pick("frame_id", "frame", "frameid")
            x_key = pick("x")
            y_key = pick("y")
            z_key = pick("z")
            yaw_key = pick("yaw_rad", "yaw", "theta", "heading")

            if all(k is not None for k in [x_key, y_key, z_key, yaw_key]) and (time_ns_key is not None or time_key is not None):
                for row in reader:
                    if time_ns_key is not None:
                        t.append(float(row[time_ns_key]) * 1.0e-9)
                    else:
                        t.append(float(row[time_key]))
                    x.append(float(row[x_key]))
                    y.append(float(row[y_key]))
                    z.append(float(row[z_key]))
                    yaw.append(float(row[yaw_key]))
                    if frame_id is None and frame_key is not None:
                        frame_id = row[frame_key]
            else:
                # Fallback: headerless legacy format: time,frame,x,y,z,yaw
                f.seek(0)
                plain_reader = csv.reader(f, dialect=dialect)
                for row in plain_reader:
                    if len(row) < 6:
                        continue
                    try:
                        t.append(float(row[0]))
                        if frame_id is None:
                            frame_id = row[1]
                        x.append(float(row[2]))
                        y.append(float(row[3]))
                        z.append(float(row[4]))
                        yaw.append(float(row[5]))
                    except ValueError:
                        continue
        else:
            raise ValueError("CSV has no readable header/rows")

    if not t:
        raise ValueError(
            "No samples found or unsupported CSV format. "
            "Expected columns like: time_sec,frame_id,x,y,z,yaw_rad"
        )

    # drop out last row that is usually corrupted
    if len(t) > 1:
        t = t[:-1]
        x = x[:-1]
        y = y[:-1]
        z = z[:-1]
        yaw = yaw[:-1]

    t0 = t[0]
    t = [v - t0 for v in t]
    return frame_id or "unknown", t, x, y, z, yaw


def segment_goal_sets(t_vals, x_vals, y_vals, gap_threshold_sec=3.0, jump_threshold_m=0.40):
    """
    Split samples into goal sets.
    Primary rule: large timestamp gap.
    Fallback (when all timestamps are identical): large XY jump.
    """
    n = len(t_vals)
    if n == 0:
        return []

    all_zero_time = max(t_vals) <= 0.0
    segments = []
    start = 0

    for i in range(1, n):
        new_segment = False

        if not all_zero_time and (t_vals[i] - t_vals[i - 1] > gap_threshold_sec):
            print("new cluster by time")
            new_segment = True
        elif all_zero_time:
            jump = math.hypot(x_vals[i] - x_vals[i - 1], y_vals[i] - y_vals[i - 1])
            print("new cluster by dist")
            if jump > jump_threshold_m:
                new_segment = True

        if new_segment:
            segments.append((start, i))
            start = i

    segments.append((start, n))
    return segments


def blend_with_white(color_rgba, darkness):
    """darkness in [0,1]: 0 -> very light, 1 -> original color"""
    r, g, b, a = color_rgba
    alpha = max(0.0, min(1.0, darkness))
    rr = (1.0 - alpha) * 1.0 + alpha * r
    gg = (1.0 - alpha) * 1.0 + alpha * g
    bb = (1.0 - alpha) * 1.0 + alpha * b
    return (rr, gg, bb, a)


def plot_history(csv_path: Path):
    frame_id, t, x, y, z, yaw = load_csv(csv_path)
    segments = segment_goal_sets(t, x, y)

    fig = plt.figure(figsize=(12.0, 7.2))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.8, 1.0])
    ax_xy = fig.add_subplot(gs[0, 0])
    ax_txt = fig.add_subplot(gs[0, 1])

    cmap_goals = plt.get_cmap("tab10")
    summary_lines = []

    for goal_idx, (s, e) in enumerate(segments, start=1):
        base = cmap_goals((goal_idx - 1) % 10)

        seg_len = e - s
        seg_x = x[s:e]
        seg_y = y[s:e]

        if seg_len <= 1:
            colors = [blend_with_white(base, 1.0)]
        else:
            colors = [
                blend_with_white(base, 0.25 + 0.75 * (k / (seg_len - 1)))
                for k in range(seg_len)
            ]

        ax_xy.scatter(
            x[s:e],
            y[s:e],
            c=colors,
            s=28,
            edgecolors="none",
            label=f"Goal {goal_idx}",
        )

        mean_x = statistics.mean(seg_x)
        mean_y = statistics.mean(seg_y)
        std_x = statistics.pstdev(seg_x) if seg_len > 1 else 0.0
        std_y = statistics.pstdev(seg_y) if seg_len > 1 else 0.0

        distances = [math.hypot(px - mean_x, py - mean_y) for px, py in zip(seg_x, seg_y)]
        std_r = statistics.pstdev(distances) if seg_len > 1 else 0.0

        ax_xy.plot(mean_x, mean_y, marker="x", markersize=9, markeredgewidth=2.0, color=base)
        ax_xy.text(mean_x + 0.02, mean_y + 0.02, f"G{goal_idx}", color=base, fontsize=10, weight="bold")

        if max(t) > 0.0:
            dt = max(0.0, t[e - 1] - t[s])
            summary_lines.append(
                f"G{goal_idx}: n={seg_len}, Δt={dt:.2f}s\n"
                f"  mean=({mean_x:.3f}, {mean_y:.3f})\n"
                f"  std=({std_x:.4f}, {std_y:.4f}), std_r={std_r:.4f}"
            )
        else:
            summary_lines.append(
                f"G{goal_idx}: n={seg_len}\n"
                f"  mean=({mean_x:.3f}, {mean_y:.3f})\n"
                f"  std=({std_x:.4f}, {std_y:.4f}), std_r={std_r:.4f}"
            )

    ax_xy.plot(x, y, alpha=0.12, linewidth=0.9, color="gray")
    ax_xy.set_title(f"Dock poses on map (frame '{frame_id}')")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_xlim(0.0, MAP_WIDTH_M)
    ax_xy.set_ylim(0.0, MAP_HEIGHT_M)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="upper right", fontsize=8)

    ax_txt.axis("off")
    title = "Goal sets (order)"
    note = "Darker points = later dock_pose (within each goal set)"
    if max(t) <= 0.0:
        title += "\n(time all zero → segmented by XY jump)"
    ax_txt.text(
        0.0,
        1.0,
        title + "\n" + note + "\n\n" + "\n\n".join(summary_lines),
        va="top",
        ha="left",
        fontsize=9,
        family="monospace",
    )

    fig.suptitle(f"Dock pose history ({len(t)} samples)")
    fig.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize dock pose history exported by SimpleChargingDock"
    )
    parser.add_argument(
        "csv_file",
        type=Path,
        nargs="?",
        default=None,
        help="CSV file path or directory (default: dock_records, newest file auto-picked)",
    )
    args = parser.parse_args()

    csv_path = resolve_csv_path(args.csv_file)
    print(f"Using CSV: {csv_path}")

    plot_history(csv_path)


if __name__ == "__main__":
    main()
