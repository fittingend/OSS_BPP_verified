#!/usr/bin/env python3
"""Compare OSS_BPP outputs against ANSWER jsonl files on a per-frame basis."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare OSS_BPP path_with_lane_id jsonl output against the ANSWER files."
    )
    parser.add_argument(
        "--scenario",
        type=int,
        choices=(1, 2, 3),
        default=1,
        help="Scenario index to compare.",
    )
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=None,
        help="Base directory that contains scenario_<n> folders (defaults to repo reference_IO).",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Render matplotlib graphs for the collected metrics.",
    )
    parser.add_argument(
        "--ignore-timestamp",
        action="store_true",
        help="Ignore timestamp/header mismatches and compare only path/point/lane data.",
    )
    return parser.parse_args()


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        raise FileNotFoundError(path)
    frames: List[Dict[str, Any]] = []
    with path.open("r") as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            frames.append(json.loads(line))
    return frames


def extract_message(entry: Dict[str, Any]) -> Dict[str, Any]:
    for key in ("message", "msg", "payload"):
        value = entry.get(key)
        if isinstance(value, dict):
            return value
    return entry


def extract_timestamp(entry: Dict[str, Any]) -> Optional[int]:
    if "timestamp" in entry:
        return entry["timestamp"]
    message = extract_message(entry)
    header = message.get("header", {})
    if "stamp" in header:
        stamp = header["stamp"]
        sec = stamp.get("sec")
        nsec = stamp.get("nanosec", 0)
        if sec is not None:
            return int(sec) * 1_000_000_000 + int(nsec)
    return None


def compare_frame_points(
    ours_points: List[Dict[str, Any]],
    answer_points: List[Dict[str, Any]],
) -> Tuple[Optional[float], bool, List[float]]:
    max_diff = None
    lane_mismatch = False
    diffs: List[float] = []
    for our_pt, ans_pt in zip(ours_points, answer_points):
        our_coords = our_pt["point"]["pose"]["position"]
        ans_coords = ans_pt["point"]["pose"]["position"]
        dx = float(our_coords["x"]) - float(ans_coords["x"])
        dy = float(our_coords["y"]) - float(ans_coords["y"])
        dz = float(our_coords["z"]) - float(ans_coords["z"])
        diff = math.sqrt(dx * dx + dy * dy + dz * dz)
        diffs.append(diff)
        max_diff = diff if max_diff is None else max(max_diff, diff)
        if our_pt.get("lane_ids", []) != ans_pt.get("lane_ids", []):
            lane_mismatch = True
    return max_diff, lane_mismatch, diffs


def _build_timestamp_index(
    frames: List[Dict[str, Any]],
) -> List[Tuple[int, int, Dict[str, Any]]]:
    indexed: List[Tuple[int, int, Dict[str, Any]]] = []
    for idx, entry in enumerate(frames):
        ts = extract_timestamp(entry)
        if ts is None:
            continue
        indexed.append((ts, idx, entry))
    indexed.sort(key=lambda item: item[0])
    return indexed


def _find_nearest_entry(
    indexed: List[Tuple[int, int, Dict[str, Any]]],
    target_ts: int,
) -> Optional[Tuple[int, int, Dict[str, Any]]]:
    import bisect

    if not indexed:
        return None
    ts_list = [item[0] for item in indexed]
    pos = bisect.bisect_left(ts_list, target_ts)
    candidates = []
    if pos < len(indexed):
        candidates.append(indexed[pos])
    if pos > 0:
        candidates.append(indexed[pos - 1])
    if not candidates:
        return None
    return min(candidates, key=lambda item: abs(item[0] - target_ts))


def collect_frame_metrics(
    ours_frames: List[Dict[str, Any]],
    answer_frames: List[Dict[str, Any]],
    ignore_timestamp: bool,
) -> Dict[str, Any]:

    max_point_diff = 0.0
    max_diff_frame = None
    timestamp_mismatches: List[Tuple[int, Optional[int], Optional[int]]] = []
    header_mismatches: List[int] = []
    point_count_mismatches: List[Tuple[int, int, int]] = []
    lane_id_mismatches: List[int] = []
    max_errors: List[float] = []
    mean_errors: List[float] = []
    end_errors: List[float] = []
    point_counts_ours: List[int] = []
    point_counts_ans: List[int] = []
    lane_mismatch_flags: List[int] = []
    frame_entries: List[Tuple[int, Optional[Dict[str, Any]], Optional[Dict[str, Any]], float]] = []

    ours_indexed = _build_timestamp_index(ours_frames)
    answer_indexed = _build_timestamp_index(answer_frames)

    # Compare based on nearest timestamp to avoid pure index misalignment.
    for idx, (ans_ts, ans_orig_idx, ans_entry) in enumerate(answer_indexed):
        nearest = _find_nearest_entry(ours_indexed, ans_ts)
        our_entry = nearest[2] if nearest else None
        our_points = []
        ans_points = []
        lane_mismatch = 0
        max_diff = 0.0
        mean_diff = 0.0
        end_diff = 0.0
        pair_count = 0

        if our_entry and ans_entry:
            our_ts = extract_timestamp(our_entry)
            if not ignore_timestamp and our_ts != ans_ts:
                timestamp_mismatches.append((idx, our_ts, ans_ts))
            our_msg = extract_message(our_entry)
            ans_msg = extract_message(ans_entry)
            if not ignore_timestamp and our_msg.get("header") != ans_msg.get("header"):
                header_mismatches.append(idx)
            our_points = our_msg.get("points", [])
            ans_points = ans_msg.get("points", [])
            point_counts_ours.append(len(our_points))
            point_counts_ans.append(len(ans_points))
            if len(our_points) != len(ans_points):
                point_count_mismatches.append((idx, len(our_points), len(ans_points)))
            max_diff, lane_flag, diffs = compare_frame_points(our_points, ans_points)
            lane_mismatch |= int(lane_flag)
            if lane_flag:
                lane_id_mismatches.append(idx)
            lane_mismatch_flags.append(int(lane_flag))
            if diffs:
                mean_diff = sum(diffs) / len(diffs)
            if our_points and ans_points:
                last_our = our_points[-1]["point"]["pose"]["position"]
                last_ans = ans_points[-1]["point"]["pose"]["position"]
                dx = float(last_our["x"]) - float(last_ans["x"])
                dy = float(last_our["y"]) - float(last_ans["y"])
                dz = float(last_our["z"]) - float(last_ans["z"])
                end_diff = math.sqrt(dx * dx + dy * dy + dz * dz)
        else:
            point_counts_ours.append(len(our_entry.get("message", {}).get("points", [])) if our_entry else 0)
            point_counts_ans.append(len(ans_entry.get("message", {}).get("points", [])) if ans_entry else 0)
            point_count_mismatches.append(
                (idx, point_counts_ours[-1], point_counts_ans[-1])
            )
            lane_mismatch_flags.append(0)

        max_errors.append(max_diff)
        mean_errors.append(mean_diff)
        end_errors.append(end_diff)
        frame_entries.append((idx, our_entry, ans_entry, max_diff))
        if max_diff > max_point_diff:
            max_point_diff = max_diff
            max_diff_frame = idx

    return {
        "max_point_diff": max_point_diff,
        "max_diff_frame": max_diff_frame,
        "timestamp_mismatches": timestamp_mismatches,
        "header_mismatches": header_mismatches,
        "point_count_mismatches": point_count_mismatches,
        "lane_id_mismatches": lane_id_mismatches,
        "max_errors": max_errors,
        "mean_errors": mean_errors,
        "end_errors": end_errors,
        "point_counts_ours": point_counts_ours,
        "point_counts_ans": point_counts_ans,
        "lane_mismatch_flags": lane_mismatch_flags,
        "frame_entries": frame_entries,
    }


def plot_error_over_time(
    errors: Dict[str, List[float]],
    scenario: int,
) -> None:
    """Plot max/mean/end point errors over frames for debugging goal/resample drift."""
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(errors["max_errors"], label="Max error")
    axes[0].set_title(f"Max Position Error per Frame (Scenario {scenario})")
    axes[0].set_ylabel("meters")
    axes[1].plot(errors["mean_errors"], label="Mean error", color="orange")
    axes[1].set_title(f"Mean Position Error per Frame (Scenario {scenario})")
    axes[1].set_ylabel("meters")
    axes[2].plot(errors["end_errors"], label="End point error", color="green")
    axes[2].set_title(f"End Point Error per Frame (Scenario {scenario})")
    axes[2].set_ylabel("meters")
    axes[2].set_xlabel("frame index")
    for ax in axes:
        ax.grid(True, linestyle="--", linewidth=0.5)
    fig.tight_layout()
    plt.show()


def plot_point_count(
    counts_ours: List[int],
    counts_ans: List[int],
    scenario: int,
) -> None:
    """Show if paths were resampled differently by comparing point counts."""
    plt.figure(figsize=(9, 4))
    plt.plot(counts_ours, label="ours")
    plt.plot(counts_ans, label="answer", linestyle="--")
    plt.title(f"Point Count per Frame (Scenario {scenario})")
    plt.ylabel("number of points")
    plt.xlabel("frame index")
    plt.legend()
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.tight_layout()
    plt.show()


def plot_lane_id_mismatch(
    lane_id_flags: List[int],
    scenario: int,
) -> None:
    """Highlight frames where lane ID assignment diverges."""
    plt.figure(figsize=(9, 2.5))
    plt.plot(lane_id_flags, drawstyle="steps-mid")
    plt.title(f"Lane ID Mismatch Flags (Scenario {scenario})")
    plt.ylabel("mismatch")
    plt.xlabel("frame index")
    plt.yticks([0, 1])
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.tight_layout()
    plt.show()


def get_xy_path(entry: Dict[str, Any]) -> Tuple[List[float], List[float]]:
    message = extract_message(entry)
    points = message.get("points", [])
    xs = [float(pt["point"]["pose"]["position"]["x"]) for pt in points]
    ys = [float(pt["point"]["pose"]["position"]["y"]) for pt in points]
    return xs, ys


def plot_xy_overlay_for_frames(
    frame_entries: List[Tuple[int, Optional[Dict[str, Any]], Optional[Dict[str, Any]], float]],
    scenario: int,
    top_n: int = 3,
) -> None:
    """Overlay the top-N error frames to inspect XY drift/overshoot."""
    sorted_entries = sorted(
        [entry for entry in frame_entries if entry[1] and entry[2]],
        key=lambda item: item[3],
        reverse=True,
    )[:top_n]
    for idx, our_entry, ans_entry, error in sorted_entries:
        our_x, our_y = get_xy_path(our_entry)
        ans_x, ans_y = get_xy_path(ans_entry)
        plt.figure(figsize=(6, 6))
        plt.plot(ans_x, ans_y, label="answer", linewidth=1.5)
        plt.plot(our_x, our_y, label="ours", linestyle="--")
        plt.title(f"XY Path Overlay – Frame {idx} (Max Error {error:.2f} m)")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.axis("equal")
        plt.legend()
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.show()


def main() -> None:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    base_dir = args.base_dir or repo_root / "reference_IO"
    scenario_dir = base_dir / f"scenario_{args.scenario}"
    if not scenario_dir.exists() or not scenario_dir.is_dir():
        raise FileNotFoundError(f"Missing scenario directory: {scenario_dir}")
    ours_path = scenario_dir / "planning__scenario_planning__lane_driving__behavior_planning__path_with_lane_id.jsonl"
    answer_path = scenario_dir / "ANSWER_planning__scenario_planning__lane_driving__behavior_planning__path_with_lane_id.jsonl"
    ours_frames = load_jsonl(ours_path)
    answer_frames = load_jsonl(answer_path)

    metrics = collect_frame_metrics(
        ours_frames, answer_frames, ignore_timestamp=args.ignore_timestamp
    )

    print(f"scenario_{args.scenario}: {len(ours_frames)} frames (ours) vs {len(answer_frames)} frames (answer)")
    print(f"max_point_diff = {metrics['max_point_diff']:.6f} (frame {metrics['max_diff_frame']})")
    if args.ignore_timestamp:
        print("timestamp mismatches: (ignored)")
        print("header/points mismatches: (ignored)")
    else:
        print("timestamp mismatches:", len(metrics["timestamp_mismatches"]))
        print("header/points mismatches:", len(metrics["header_mismatches"]))
    print("point count mismatches:", len(metrics["point_count_mismatches"]))
    print("lane id mismatches:", len(metrics["lane_id_mismatches"]))
    if not args.ignore_timestamp and metrics["timestamp_mismatches"]:
        print("first timestamp mismatch:", metrics["timestamp_mismatches"][0])

    if args.plot:
        plot_error_over_time(
            {
                "max_errors": metrics["max_errors"],
                "mean_errors": metrics["mean_errors"],
                "end_errors": metrics["end_errors"],
            },
            args.scenario,
        )
        plot_point_count(metrics["point_counts_ours"], metrics["point_counts_ans"], args.scenario)
        plot_lane_id_mismatch(metrics["lane_mismatch_flags"], args.scenario)
        plot_xy_overlay_for_frames(metrics["frame_entries"], args.scenario)


if __name__ == "__main__":
    main()
