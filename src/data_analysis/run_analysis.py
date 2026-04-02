#!/usr/bin/env python3
"""Generate RMSE tables and plots from one or more controller log CSVs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import utilities


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run non-interactive analysis on one or more controller log CSVs."
    )
    parser.add_argument(
        "--log",
        dest="logs",
        action="append",
        default=[],
        help="Path to a CSV log file. Repeat for multiple logs.",
    )
    parser.add_argument(
        "--log-dir",
        action="append",
        default=[],
        help="Directory containing CSV logs. Repeat for multiple directories.",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory where tables and plots should be written.",
    )
    return parser.parse_args()


def collect_logs(args: argparse.Namespace) -> list[Path]:
    logs: list[Path] = [Path(path).resolve() for path in args.logs]
    for directory in args.log_dir:
        logs.extend(sorted(Path(directory).resolve().glob("*.csv")))

    deduped: list[Path] = []
    seen: set[Path] = set()
    for log in logs:
        if log in seen:
            continue
        if not log.is_file():
            raise FileNotFoundError(f"Log file not found: {log}")
        deduped.append(log)
        seen.add(log)

    if not deduped:
        raise ValueError("No CSV logs were provided.")
    return deduped


def load_data(logs: list[Path]) -> tuple[dict[str, object], dict[str, Path]]:
    data_dict: dict[str, object] = {}
    path_map: dict[str, Path] = {}
    for log in logs:
        key = f"{log.parent.name}__{log.name}"
        data_dict[key] = utilities.load_csv(str(log))
        path_map[key] = log
    return data_dict, path_map


def save_summary_tables(results_df, output_dir: Path) -> None:
    results_df.to_csv(output_dir / "summary.csv", index=False)
    (output_dir / "summary.md").write_text(dataframe_to_markdown(results_df))
    (output_dir / "summary.tex").write_text(utilities.format_latex_table(results_df))


def dataframe_to_markdown(df) -> str:
    columns = list(df.columns)
    lines = [
        "| " + " | ".join(str(column) for column in columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for _, row in df.iterrows():
        lines.append("| " + " | ".join(str(row[column]) for column in columns) + " |")
    return "\n".join(lines) + "\n"


def plot_single_log(key: str, df, output_dir: Path) -> dict[str, object]:
    metadata = utilities.extract_metadata_from_data(df)
    position_rmse = utilities.calculate_position_rmse(df)
    overall_rmse = utilities.calculate_overall_rmse(df)
    axis_rmse = utilities.calculate_rmse_per_axis(df)

    utilities.setup_publication_style()
    fig, ax = plt.subplots(figsize=(6, 5))
    utilities.plot_trajectory_2d(ax, df, actual_label="Actual", ref_label="Reference")
    ax.set_title(
        f"{metadata['controller']} | {metadata['trajectory']}\n"
        f"Position RMSE: {position_rmse:.3f} m"
    )
    fig.savefig(output_dir / f"{Path(key).stem}_trajectory.pdf")
    plt.close(fig)

    utilities.plot_time_series(
        df,
        vars_to_plot=["x", "y", "z", "yaw"],
        save_path=str(output_dir / f"{Path(key).stem}_timeseries.pdf"),
    )
    plt.close("all")

    return {
        "key": key,
        "controller": metadata["controller"],
        "trajectory": metadata["trajectory"],
        "platform": metadata["platform"],
        "position_rmse_m": float(position_rmse),
        "overall_rmse": float(overall_rmse),
        "axis_rmse": {axis: float(value) for axis, value in axis_rmse.items()},
    }


def save_overlay_plot(data_dict: dict[str, object], output_dir: Path) -> Path | None:
    if len(data_dict) < 2:
        return None

    first_df = next(iter(data_dict.values()))
    plane = utilities.detect_trajectory_plane(first_df)
    aligned = {
        key: utilities.align_reference_to_actual(df)
        for key, df in data_dict.items()
    }

    utilities.setup_publication_style()
    fig, ax = plt.subplots(figsize=(7, 6))
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    x_col, y_col, x_ref_col, y_ref_col, xlabel, ylabel, z_flip = plane_columns(plane)
    first_key = next(iter(aligned.keys()))
    ref_df = aligned[first_key]
    ax.plot(
        ref_df[x_ref_col],
        ref_df[y_ref_col] * z_flip,
        linestyle="--",
        color="black",
        linewidth=2,
        label="Reference",
    )

    for idx, (key, df) in enumerate(aligned.items()):
        color = colors[idx % len(colors)]
        ax.plot(df[x_col], df[y_col] * z_flip, color=color, label=key.replace("__", ": "))

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("Controller trajectory overlay")
    output_path = output_dir / "comparison_overlay.pdf"
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def validate_summary_metadata(results_df):
    invalid_rows = results_df[
        (results_df["Platform"] == "Unknown")
        | (results_df["Controller"].isin(["Unknown", "Controller 2"]))
        | (results_df["Trajectory"] == "Unknown")
    ]
    if invalid_rows.empty:
        return

    bad_rows = invalid_rows[["Platform", "Controller", "Trajectory", "Modifiers"]].drop_duplicates()
    raise RuntimeError(
        "Analysis produced unresolved metadata rows:\n"
        f"{bad_rows.to_string(index=False)}"
    )


def plane_columns(plane: str) -> tuple[str, str, str, str, str, str, int]:
    if plane == "xy":
        return "x", "y", "x_ref", "y_ref", "x [m]", "y [m]", 1
    if plane == "xz":
        return "x", "z", "x_ref", "z_ref", "x [m]", "z [m]", -1
    if plane == "yz":
        return "y", "z", "y_ref", "z_ref", "y [m]", "z [m]", -1
    raise ValueError(f"Unsupported trajectory plane: {plane}")


def main() -> None:
    args = parse_args()
    logs = collect_logs(args)
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    data_dict, path_map = load_data(logs)
    results_df = utilities.generate_results_table(data_dict)
    validate_summary_metadata(results_df)
    save_summary_tables(results_df, output_dir)

    manifest = {
        "logs": {key: str(path_map[key]) for key in data_dict},
        "artifacts": {
            "summary_csv": str(output_dir / "summary.csv"),
            "summary_md": str(output_dir / "summary.md"),
            "summary_tex": str(output_dir / "summary.tex"),
        },
        "per_log": [],
    }

    for key, df in data_dict.items():
        manifest["per_log"].append(plot_single_log(key, df, output_dir))

    overlay_path = save_overlay_plot(data_dict, output_dir)
    if overlay_path is not None:
        manifest["artifacts"]["comparison_overlay"] = str(overlay_path)
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    print(f"[analysis] Wrote results to {output_dir}")


if __name__ == "__main__":
    main()
