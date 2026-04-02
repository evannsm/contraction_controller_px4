#!/usr/bin/env python3
"""Render comparison plots for the completed Agents.md sweep."""

from __future__ import annotations

import argparse
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--summary-dir",
        default=str(REPO_ROOT / "docs" / "generated" / "agents_matrix_20260401"),
        help="Directory containing the tracked aggregate CSV outputs.",
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

    fig, ax = plt.subplots(figsize=(max(14, n_cols * 0.82), max(4.6, n_rows * 0.82)))
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
            normalized = norm(value)
            text_color = "black" if normalized > 0.58 else "white"
            ax.text(
                col_idx,
                row_idx,
                format_metric(value),
                ha="center",
                va="center",
                color=text_color,
                fontsize=9,
            )

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
    fig, ax = plt.subplots(figsize=(10, 4.8))
    bars = ax.barh(best_rows["trajectory_display"], best_rows["Position_RMSE_m"], color="#2962a3")
    ax.set_title("Best Controller Per Trajectory")
    ax.set_xlabel("Position RMSE [m]")
    ax.set_ylabel("Trajectory")
    ax.invert_yaxis()
    for bar, label in zip(bars, best_rows["comparison_label_final"]):
        ax.text(
            bar.get_width() + 0.01,
            bar.get_y() + bar.get_height() / 2.0,
            f"{label} ({bar.get_width():.3f} m)",
            va="center",
            fontsize=10,
        )
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
    fig, axes = plt.subplots(n_facets, 1, figsize=(11, max(4.8, n_facets * 4.2)), sharex=False)
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
            ax.text(x_pos, bar.get_y() + bar.get_height() / 2.0, f"{value:.2f}%", va="center", ha=ha, fontsize=9)

    fig.suptitle(title, y=1.01)
    fig.tight_layout()
    save_figure(fig, output_dir, stem)


def render_fbl_tuning(summary: pd.DataFrame, output_dir: Path) -> None:
    utilities.setup_publication_style()
    ordered = summary.sort_values("mean_position_rmse_m", ascending=True).reset_index(drop=True)
    colors = ["#ef6c00" if profile == "aggressive" else "#546e7a" for profile in ordered["fbl_gain_profile"]]

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    axes[0].bar(ordered["fbl_gain_profile"], ordered["mean_position_rmse_m"], color=colors)
    axes[0].set_title("FBL Tuning: Mean RMSE")
    axes[0].set_ylabel("Mean position RMSE [m]")

    axes[1].bar(ordered["fbl_gain_profile"], ordered["mean_comp_time_ms"], color=colors)
    axes[1].set_title("FBL Tuning: Mean Compute Time")
    axes[1].set_ylabel("Mean compute time [ms]")

    for ax, column in zip(axes, ("mean_position_rmse_m", "mean_comp_time_ms")):
        for patch, value in zip(ax.patches, ordered[column]):
            ax.text(
                patch.get_x() + patch.get_width() / 2.0,
                patch.get_height(),
                format_metric(float(value)),
                ha="center",
                va="bottom",
                fontsize=10,
            )

    fig.tight_layout()
    save_figure(fig, output_dir, "fbl_tuning_profiles")


def render_cpp_acceptance(summary: pd.DataFrame, output_dir: Path) -> None:
    utilities.setup_publication_style()
    df = summary.copy()
    df["controller"] = df["comparison_label"].str.replace(" baseline", "", regex=False).str.replace(" workshop", "", regex=False)
    df["profile"] = df["comparison_label"].str.extract(r"(baseline|workshop)")
    pivot = df.pivot(index="controller", columns="profile", values="Position_RMSE_m").reindex(
        ["Standard NR C++", "NR Enhanced C++", "NR Diff-Flat C++"]
    )

    fig, ax = plt.subplots(figsize=(9.5, 4.8))
    x = np.arange(len(pivot.index))
    width = 0.34
    baseline = pivot["baseline"].to_numpy(dtype=float)
    workshop = pivot["workshop"].to_numpy(dtype=float)
    ax.bar(x - width / 2.0, baseline, width=width, label="baseline", color="#78909c")
    ax.bar(x + width / 2.0, workshop, width=width, label="workshop", color="#1565c0")
    ax.set_xticks(x)
    ax.set_xticklabels(pivot.index, rotation=12, ha="right")
    ax.set_ylabel("Position RMSE [m]")
    ax.set_title("C++ Acceptance on Fig8 Contraction")
    ax.legend()

    for xpos, base, work in zip(x, baseline, workshop):
        delta_pct = ((work - base) / base) * 100.0
        ax.text(xpos, max(base, work), f"{delta_pct:+.1f}%", ha="center", va="bottom", fontsize=10)

    fig.tight_layout()
    save_figure(fig, output_dir, "cpp_acceptance_rmse")


def main() -> None:
    args = parse_args()
    summary_dir = Path(args.summary_dir).resolve()
    output_dir = Path(args.output_dir).resolve() if args.output_dir else summary_dir / "plots"

    position_rmse = pd.read_csv(summary_dir / "position_rmse_table.csv")
    comp_time = pd.read_csv(summary_dir / "comp_time_table.csv")
    nr_improvements = pd.read_csv(summary_dir / "nr_workshop_improvements.csv")
    feedforward_effects = pd.read_csv(summary_dir / "feedforward_effects.csv")
    fbl_tuning = pd.read_csv(summary_dir / "fbl_tuning_summary.csv")
    cpp_acceptance = pd.read_csv(summary_dir / "cpp_acceptance_summary.csv")
    main_matrix = pd.read_csv(summary_dir / "agents_main_matrix.csv")

    render_best_by_trajectory(main_matrix, output_dir)
    render_heatmap(
        position_rmse,
        title="Agents.md Sweep: Position RMSE Across Controllers and Trajectories",
        metric_label="Position RMSE [m]",
        output_dir=output_dir,
        stem="position_rmse_heatmap",
        log_scale=True,
    )
    render_heatmap(
        comp_time,
        title="Agents.md Sweep: Compute Time Across Controllers and Trajectories",
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
        title="NR Workshop Profile Change vs Baseline",
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
        title="Feedforward Effect by Controller Family",
        xlabel="ff minus noff [%]",
        output_dir=output_dir,
        stem="feedforward_effect_delta_pct",
    )
    render_fbl_tuning(fbl_tuning, output_dir)
    render_cpp_acceptance(cpp_acceptance, output_dir)

    print(f"[plot-agents-matrix] Wrote plots to {output_dir}")


if __name__ == "__main__":
    main()
