#!/usr/bin/env python3
"""Render plots for the regular trajectories above fig8_contraction."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import LogNorm, Normalize


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "data_analysis"))
import utilities  # noqa: E402


TRAJECTORY_ORDER = [
    "Hover",
    "Yaw Only",
    "Circle Horizontal",
    "Circle Vertical",
    "Fig8 Horizontal",
    "Fig8 Vertical",
]

REFERENCE_PLOT_PLANES = {
    "Hover": "xz",
    "Yaw Only": "xz",
    "Circle Horizontal": "xy",
    "Circle Vertical": "xz",
    "Fig8 Horizontal": "xy",
    "Fig8 Vertical": "xz",
}

COMPARISON_ORDER = [
    "Standard NR baseline noff",
    "Standard NR baseline ff",
    "Standard NR workshop noff",
    "Standard NR workshop ff",
    "NR Enhanced baseline noff",
    "NR Enhanced baseline ff",
    "NR Enhanced workshop noff",
    "NR Enhanced workshop ff",
    "NR Diff-Flat baseline noff",
    "NR Diff-Flat baseline ff",
    "NR Diff-Flat workshop noff",
    "NR Diff-Flat workshop ff",
    "NMPC noff",
    "NMPC ff",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--summary-dir",
        default=str(REPO_ROOT / "src" / "data_analysis" / "results" / "pre_fig8_matrix_20260402" / "aggregate"),
        help="Directory containing aggregate CSV outputs from run_pre_fig8_matrix.py.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for the generated plots. Defaults to <summary-dir>/plots.",
    )
    return parser.parse_args()


def save_figure(fig: plt.Figure, output_dir: Path, stem: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / f"{stem}.pdf")
    fig.savefig(output_dir / f"{stem}.png", dpi=300)
    plt.close(fig)


def format_metric(value: float) -> str:
    if not math.isfinite(value):
        return "-"
    if abs(value) >= 100.0:
        return f"{value:.1f}"
    if abs(value) >= 10.0:
        return f"{value:.2f}"
    return f"{value:.3f}"


def render_heatmap(
    table: pd.DataFrame,
    *,
    title: str,
    metric_label: str,
    output_dir: Path,
    stem: str,
    log_scale: bool = False,
) -> None:
    utilities.setup_publication_style()
    df = table.set_index("trajectory_display")
    values = df.to_numpy(dtype=float)
    n_rows, n_cols = values.shape

    fig, ax = plt.subplots(figsize=(max(12, n_cols * 0.82), max(4.5, n_rows * 0.82)))
    positive_values = values[np.isfinite(values) & (values > 0.0)]
    if log_scale:
        norm = LogNorm(vmin=float(np.min(positive_values)), vmax=float(np.max(positive_values)))
    else:
        norm = Normalize(vmin=float(np.min(values)), vmax=float(np.max(values)))

    image = ax.imshow(values, cmap="viridis_r", aspect="auto", norm=norm)
    ax.set_title(title)
    ax.set_xlabel("Controller / configuration")
    ax.set_ylabel("Trajectory")
    ax.set_xticks(np.arange(n_cols))
    ax.set_xticklabels(df.columns, rotation=55, ha="right")
    ax.set_yticks(np.arange(n_rows))
    ax.set_yticklabels(df.index)

    for row_idx in range(n_rows):
        for col_idx in range(n_cols):
            value = values[row_idx, col_idx]
            text_color = "black" if norm(value) > 0.58 else "white"
            ax.text(col_idx, row_idx, format_metric(value), ha="center", va="center", color=text_color, fontsize=8)

    cbar = fig.colorbar(image, ax=ax, shrink=0.88)
    cbar.set_label(metric_label)
    fig.tight_layout()
    save_figure(fig, output_dir, stem)


def render_best_by_trajectory(main_matrix: pd.DataFrame, output_dir: Path) -> None:
    utilities.setup_publication_style()
    best_rows = (
        main_matrix.sort_values("Position_RMSE_m")
        .groupby("trajectory_display", as_index=False)
        .first()
        .sort_values("Position_RMSE_m", ascending=True)
    )
    fig, ax = plt.subplots(figsize=(10, 5.0))
    bars = ax.barh(best_rows["trajectory_display"], best_rows["Position_RMSE_m"], color="#2962a3")
    ax.set_title("Best Controller Per Pre-Fig8 Trajectory")
    ax.set_xlabel("Position RMSE [m]")
    ax.set_ylabel("Trajectory")
    ax.invert_yaxis()
    for bar, label in zip(bars, best_rows["comparison_label_final"]):
        ax.text(bar.get_width() + 0.01, bar.get_y() + bar.get_height() / 2.0, f"{label} ({bar.get_width():.3f} m)", va="center", fontsize=9)
    fig.tight_layout()
    save_figure(fig, output_dir, "best_by_trajectory_rmse")


def render_signed_delta_plot(
    df: pd.DataFrame,
    *,
    facet_col: str,
    facet_order: list[str],
    value_col: str,
    title: str,
    xlabel: str,
    output_dir: Path,
    stem: str,
) -> None:
    utilities.setup_publication_style()
    n_facets = len(facet_order)
    fig, axes = plt.subplots(n_facets, 1, figsize=(11, max(4.8, n_facets * 4.0)), sharex=False)
    if n_facets == 1:
        axes = [axes]

    for ax, facet_value in zip(axes, facet_order):
        subset = df[df[facet_col] == facet_value].copy()
        if subset.empty:
            ax.axis("off")
            continue
        subset = subset.sort_values(value_col, ascending=True)
        labels = [f"{row.trajectory_display} | {row.controller}" for row in subset.itertuples()]
        colors = ["#2e7d32" if value < 0.0 else "#c62828" for value in subset[value_col]]
        bars = ax.barh(labels, subset[value_col], color=colors)
        ax.axvline(0.0, color="black", linewidth=1.0)
        ax.set_title(str(facet_value))
        ax.set_xlabel(xlabel)
        for bar, value in zip(bars, subset[value_col]):
            offset = max(abs(value) * 0.01, 1.0)
            x_pos = value + offset if value >= 0 else value - offset
            ha = "left" if value >= 0 else "right"
            ax.text(x_pos, bar.get_y() + bar.get_height() / 2.0, f"{value:.2f}%", va="center", ha=ha, fontsize=8)

    fig.suptitle(title, y=1.01)
    fig.tight_layout()
    save_figure(fig, output_dir, stem)


def resolve_log_path(row: pd.Series) -> Path:
    def to_host_path(path_text: str) -> Path:
        path = Path(path_text)
        if path.exists():
            return path
        if path_text.startswith("/workspace/"):
            host_path = REPO_ROOT / path_text.removeprefix("/workspace/")
            if host_path.exists():
                return host_path
        return path

    manifest_path = Path(row["results_dir"]) / "analysis" / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    controller_key = row["controller_key"]
    desired_label = str(row["Controller"]).strip()
    for log_path in manifest.get("logs", {}).values():
        resolved_log_path = to_host_path(log_path)
        df = utilities.load_csv(str(resolved_log_path))
        metadata = utilities.extract_metadata_from_data(df)
        if utilities.extract_metadata_from_data(df)["controller"] == desired_label:
            return resolved_log_path
        normalized = {
            "NR Standard": "newton_raphson",
            "NR Enhanced": "newton_raphson_enhanced",
            "NR Diff-Flat": "nr_diff_flat",
            "NMPC": "nmpc",
        }.get(metadata["controller"], metadata["controller"].lower().replace(" ", "_"))
        if normalized == controller_key:
            return resolved_log_path
    raise FileNotFoundError(f"Could not resolve raw log for {row['comparison_label_final']} in {row['results_dir']}")


def render_reference_grids(main_matrix: pd.DataFrame, output_dir: Path) -> None:
    grids_dir = output_dir / "reference_vs_actual"
    grids_dir.mkdir(parents=True, exist_ok=True)

    ordered = main_matrix.copy()
    ordered["comparison_order"] = ordered["comparison_label_final"].map({label: idx for idx, label in enumerate(COMPARISON_ORDER)})

    for trajectory in TRAJECTORY_ORDER:
        traj_rows = ordered[ordered["trajectory_display"] == trajectory].sort_values("comparison_order")
        if traj_rows.empty:
            continue

        n = len(traj_rows)
        n_cols = 3
        n_rows = int(math.ceil(n / n_cols))
        plane = REFERENCE_PLOT_PLANES.get(trajectory)
        utilities.setup_publication_style()
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(n_cols * 4.8, n_rows * 4.2), squeeze=False)

        for ax, (_, row) in zip(axes.flatten(), traj_rows.iterrows()):
            log_path = resolve_log_path(row)
            df = utilities.align_reference_to_actual(utilities.load_csv(str(log_path)))
            utilities.plot_trajectory_2d(
                ax,
                df,
                plane=plane,
                actual_label="Actual",
                ref_label="Reference",
            )
            title_suffix = plane.upper() if plane else "auto"
            ax.set_title(
                f"{row['comparison_label_final']}\nRMSE {row['Position_RMSE_m']:.3f} m ({title_suffix})",
                fontsize=9,
            )
            handles, labels = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=7, loc="best")

        for ax in axes.flatten()[n:]:
            ax.axis("off")

        fig.suptitle(f"{trajectory}: reference vs actual", y=1.01)
        fig.tight_layout()
        stem = trajectory.lower().replace(" ", "_")
        save_figure(fig, grids_dir, f"{stem}_reference_vs_actual")


def main() -> None:
    args = parse_args()
    summary_dir = Path(args.summary_dir).resolve()
    output_dir = Path(args.output_dir).resolve() if args.output_dir else summary_dir / "plots"

    position_rmse = pd.read_csv(summary_dir / "position_rmse_table.csv")
    comp_time = pd.read_csv(summary_dir / "comp_time_table.csv")
    nr_improvements = pd.read_csv(summary_dir / "nr_workshop_improvements.csv")
    feedforward_effects = pd.read_csv(summary_dir / "feedforward_effects.csv")
    main_matrix = pd.read_csv(summary_dir / "pre_fig8_main_matrix.csv")
    raw_rows = pd.read_csv(summary_dir / "raw_summary_rows.csv")
    if "comparison_label_final" not in raw_rows.columns:
        raw_rows["comparison_label_final"] = raw_rows["comparison_label"]

    render_best_by_trajectory(main_matrix, output_dir)
    render_heatmap(
        position_rmse,
        title="Pre-Fig8 Matrix: Position RMSE Across Regular Trajectories",
        metric_label="Position RMSE [m]",
        output_dir=output_dir,
        stem="position_rmse_heatmap",
        log_scale=True,
    )
    render_heatmap(
        comp_time,
        title="Pre-Fig8 Matrix: Compute Time Across Regular Trajectories",
        metric_label="Compute time [ms]",
        output_dir=output_dir,
        stem="comp_time_heatmap",
        log_scale=False,
    )
    render_signed_delta_plot(
        nr_improvements,
        facet_col="ff_mode",
        facet_order=["noff", "ff"],
        value_col="delta_pct",
        title="Pre-Fig8 NR Workshop Delta vs Baseline",
        xlabel="Workshop delta vs baseline [%]",
        output_dir=output_dir,
        stem="nr_workshop_delta_pct",
    )
    render_signed_delta_plot(
        feedforward_effects.assign(
            facet_group=feedforward_effects["nr_profile"].replace({"-": "fixed references"})
        ),
        facet_col="facet_group",
        facet_order=["fixed references", "baseline", "workshop"],
        value_col="delta_ff_minus_noff_pct",
        title="Pre-Fig8 Feedforward Effect",
        xlabel="ff minus noff [%]",
        output_dir=output_dir,
        stem="feedforward_effect_delta_pct",
    )
    render_reference_grids(raw_rows, output_dir)
    print(f"[plot-pre-fig8-matrix] Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()
