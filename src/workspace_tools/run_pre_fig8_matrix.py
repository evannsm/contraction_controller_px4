#!/usr/bin/env python3
"""Run and aggregate the regular-trajectory matrix above fig8_contraction."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

import pandas as pd


TRAJECTORIES = (
    "hover",
    "yaw_only",
    "circle_horz",
    "circle_vert",
    "fig8_horz",
    "fig8_vert",
)

TRAJECTORY_LABELS = {
    "hover": "Hover",
    "yaw_only": "Yaw Only",
    "circle_horz": "Circle Horizontal",
    "circle_vert": "Circle Vertical",
    "fig8_horz": "Fig8 Horizontal",
    "fig8_vert": "Fig8 Vertical",
}

NR_CONTROLLER_LABELS = {
    "newton_raphson": "Standard NR",
    "newton_raphson_enhanced": "NR Enhanced",
    "nr_diff_flat": "NR Diff-Flat",
}

SUMMARY_CONTROLLER_TO_KEY = {
    "NR Standard": "newton_raphson",
    "NR Enhanced": "newton_raphson_enhanced",
    "NR Diff-Flat": "nr_diff_flat",
    "NR Diff-Flat (JAX)": "nr_diff_flat",
    "NR Diff-Flat (NumPy)": "nr_diff_flat",
    "NMPC": "nmpc",
    "MPC": "nmpc",
}

REQUESTED_CONTROLLER_TO_LABEL = {
    "newton_raphson": "NR Standard",
    "newton_raphson_enhanced": "NR Enhanced",
    "nr_diff_flat": "NR Diff-Flat",
    "nmpc": "NMPC",
}


@dataclass(frozen=True)
class SweepCase:
    name: str
    controllers: tuple[str, ...]
    nr_profile: str = "baseline"
    ff: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace-root",
        default=str(Path(__file__).resolve().parents[2]),
        help="Workspace root. Defaults to the current repository root.",
    )
    parser.add_argument(
        "--results-tag",
        default=f"pre_fig8_matrix_{datetime.now().strftime('%Y%m%d')}",
        help="Top-level results directory name under src/data_analysis/results/.",
    )
    parser.add_argument(
        "--px4-dir",
        default=str(Path.home() / "PX4-Autopilot"),
        help="PX4-Autopilot path to forward into fly_pipeline.py.",
    )
    parser.add_argument(
        "--reuse-existing",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reuse existing per-case results if analysis/summary.csv already exists.",
    )
    return parser.parse_args()


def markdown_table(df: pd.DataFrame) -> str:
    columns = list(df.columns)
    lines = [
        "| " + " | ".join(str(column) for column in columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for _, row in df.iterrows():
        lines.append("| " + " | ".join(str(row[column]) for column in columns) + " |")
    return "\n".join(lines) + "\n"


def results_root(workspace_root: Path, results_tag: str) -> Path:
    return workspace_root / "src" / "data_analysis" / "results" / results_tag


def normalize_controller_key(controller_label: str) -> str:
    return SUMMARY_CONTROLLER_TO_KEY.get(controller_label, controller_label.lower().replace(" ", "_"))


def modifiers_set(modifier_text: str) -> set[str]:
    if not isinstance(modifier_text, str) or modifier_text == "-":
        return set()
    return {token for token in modifier_text.split("+") if token}


def comparison_label(row: pd.Series) -> str:
    controller_key = row["controller_key"]
    modifiers = modifiers_set(row["Modifiers"])

    if controller_key in NR_CONTROLLER_LABELS:
        family = NR_CONTROLLER_LABELS[controller_key]
        nr_profile = "workshop" if "workshop" in modifiers else "baseline"
        ff_mode = "ff" if "ff" in modifiers else "noff"
        return f"{family} {nr_profile} {ff_mode}"

    if controller_key == "nmpc":
        ff_mode = "ff" if "ff" in modifiers else "noff"
        return f"NMPC {ff_mode}"

    return str(row["Controller"])


def validate_summary_metadata(df: pd.DataFrame, record: dict[str, Any]) -> None:
    if df.empty:
        raise RuntimeError(
            f"Summary is empty for {record['trajectory_key']}/{record['case_name']}"
        )

    if "Controller" not in df.columns:
        raise RuntimeError(
            f"Summary is missing Controller column for {record['trajectory_key']}/{record['case_name']}"
        )

    controller_labels = {
        str(value).strip()
        for value in df["Controller"].dropna().unique()
        if str(value).strip()
    }
    unresolved = controller_labels.intersection({"Unknown", "Controller 2"})
    if unresolved:
        raise RuntimeError(
            "Summary contains unresolved controller metadata "
            f"for {record['trajectory_key']}/{record['case_name']}: {sorted(unresolved)}"
        )

    normalized_keys = {normalize_controller_key(label) for label in controller_labels}
    requested_keys = set(record["controllers"])
    missing = requested_keys - normalized_keys
    unexpected = normalized_keys - requested_keys
    if missing or unexpected:
        raise RuntimeError(
            "Summary controller labels do not match the requested controller set "
            f"for {record['trajectory_key']}/{record['case_name']}: "
            f"requested={sorted(requested_keys)} actual={sorted(normalized_keys)}"
        )


def augment_rows(df: pd.DataFrame, record: dict[str, Any]) -> pd.DataFrame:
    augmented = df.copy()
    for key, value in record.items():
        augmented[key] = value

    requested_controllers = record["controllers"]
    if len(requested_controllers) == 1:
        requested_key = requested_controllers[0]
        requested_label = REQUESTED_CONTROLLER_TO_LABEL.get(requested_key, requested_key)
        augmented.loc[augmented["Controller"].isin(["Unknown", "Controller 2"]), "Controller"] = requested_label

    augmented["controller_key"] = augmented["Controller"].map(normalize_controller_key)
    augmented["comparison_label"] = augmented.apply(comparison_label, axis=1)
    augmented["trajectory_display"] = augmented["trajectory_key"].map(TRAJECTORY_LABELS)
    return augmented


def write_table(df: pd.DataFrame, csv_path: Path, md_path: Path, float_cols: tuple[str, ...] = ()) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    output_df = df.copy()
    for col in float_cols:
        if col in output_df.columns:
            output_df[col] = output_df[col].map(lambda value: round(float(value), 6) if pd.notna(value) else value)
    output_df.to_csv(csv_path, index=False)
    md_path.write_text(markdown_table(output_df))


def run_case(
    *,
    workspace_root: Path,
    px4_dir: Path,
    results_dir: Path,
    trajectory: str,
    case: SweepCase,
    reuse_existing: bool,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    summary_path = results_dir / "analysis" / "summary.csv"
    record: dict[str, Any] = {
        "trajectory_key": trajectory,
        "trajectory_label": TRAJECTORY_LABELS[trajectory],
        "case_name": case.name,
        "controllers": list(case.controllers),
        "nr_profile_requested": case.nr_profile,
        "ff_requested": case.ff,
        "results_dir": str(results_dir),
        "summary_csv": str(summary_path),
    }

    if reuse_existing and summary_path.is_file():
        record["status"] = "reused"
        df = pd.read_csv(summary_path)
        validate_summary_metadata(df, record)
        return df, record

    cmd = [
        sys.executable,
        str(workspace_root / "src" / "workspace_tools" / "fly_pipeline.py"),
        "--workspace-root",
        str(workspace_root),
        "--platform",
        "sim",
        "--trajectory",
        trajectory,
        "--controllers",
        ",".join(case.controllers),
        "--headless",
        "--px4-dir",
        str(px4_dir),
        "--results-dir",
        str(results_dir),
    ]

    if trajectory == "hover":
        cmd.extend(["--hover-mode", "1"])
    if case.nr_profile:
        cmd.extend(["--nr-profile", case.nr_profile])
    if case.ff:
        cmd.append("--ff")

    print(
        f"[pre-fig8-matrix] {trajectory} :: {case.name} "
        f"({', '.join(case.controllers)})"
    )
    result = subprocess.run(cmd, cwd=workspace_root)
    if result.returncode != 0:
        record["status"] = "failed"
        raise RuntimeError(f"Command failed for {trajectory}/{case.name}")

    if not summary_path.is_file():
        raise FileNotFoundError(f"Expected analysis summary was not created: {summary_path}")

    record["status"] = "completed"
    df = pd.read_csv(summary_path)
    validate_summary_metadata(df, record)
    return df, record


def run_matrix(
    *,
    workspace_root: Path,
    px4_dir: Path,
    root: Path,
    reuse_existing: bool,
    cases: list[SweepCase],
) -> tuple[pd.DataFrame, list[dict[str, Any]]]:
    rows: list[pd.DataFrame] = []
    records: list[dict[str, Any]] = []

    for trajectory in TRAJECTORIES:
        for case in cases:
            case_dir = root / "main_matrix" / trajectory / case.name
            df, record = run_case(
                workspace_root=workspace_root,
                px4_dir=px4_dir,
                results_dir=case_dir,
                trajectory=trajectory,
                case=case,
                reuse_existing=reuse_existing,
            )
            records.append(record)
            rows.append(augment_rows(df, record))

    return pd.concat(rows, ignore_index=True), records


def build_nr_improvement_table(matrix_df: pd.DataFrame) -> pd.DataFrame:
    nr_df = matrix_df[matrix_df["controller_key"].isin(set(NR_CONTROLLER_LABELS))].copy()
    nr_df["ff_mode"] = nr_df["Modifiers"].map(lambda mod: "ff" if "ff" in modifiers_set(mod) else "noff")
    nr_df["nr_profile"] = nr_df["Modifiers"].map(lambda mod: "workshop" if "workshop" in modifiers_set(mod) else "baseline")

    pivot = nr_df.pivot_table(
        index=["trajectory_key", "trajectory_display", "controller_key", "ff_mode"],
        columns="nr_profile",
        values="Position_RMSE_m",
        aggfunc="first",
    ).reset_index()
    if "baseline" not in pivot.columns or "workshop" not in pivot.columns:
        return pivot

    pivot["delta_m"] = pivot["workshop"] - pivot["baseline"]
    pivot["delta_pct"] = 100.0 * pivot["delta_m"] / pivot["baseline"]
    pivot["controller"] = pivot["controller_key"].map(NR_CONTROLLER_LABELS)
    return pivot[
        [
            "trajectory_key",
            "trajectory_display",
            "controller",
            "ff_mode",
            "baseline",
            "workshop",
            "delta_m",
            "delta_pct",
        ]
    ].sort_values(["trajectory_key", "controller", "ff_mode"])


def build_feedforward_effect_table(matrix_df: pd.DataFrame) -> pd.DataFrame:
    ff_rows: list[dict[str, Any]] = []

    for controller_key in (*NR_CONTROLLER_LABELS.keys(), "nmpc"):
        subset = matrix_df[matrix_df["controller_key"] == controller_key].copy()
        if subset.empty:
            continue
        subset["nr_profile"] = subset["Modifiers"].map(
            lambda mod: "workshop" if "workshop" in modifiers_set(mod) else (
                "baseline" if controller_key in NR_CONTROLLER_LABELS else "-"
            )
        )
        subset["ff_mode"] = subset["Modifiers"].map(lambda mod: "ff" if "ff" in modifiers_set(mod) else "noff")
        index_cols = ["trajectory_key", "trajectory_display", "nr_profile"]
        pivot = subset.pivot_table(
            index=index_cols,
            columns="ff_mode",
            values="Position_RMSE_m",
            aggfunc="first",
        ).reset_index()
        if "ff" not in pivot.columns or "noff" not in pivot.columns:
            continue
        pivot["controller"] = "NMPC" if controller_key == "nmpc" else NR_CONTROLLER_LABELS[controller_key]
        pivot["delta_ff_minus_noff_m"] = pivot["ff"] - pivot["noff"]
        pivot["delta_ff_minus_noff_pct"] = 100.0 * pivot["delta_ff_minus_noff_m"] / pivot["noff"]
        ff_rows.extend(pivot.to_dict("records"))

    return pd.DataFrame(ff_rows)[
        [
            "trajectory_key",
            "trajectory_display",
            "controller",
            "nr_profile",
            "noff",
            "ff",
            "delta_ff_minus_noff_m",
            "delta_ff_minus_noff_pct",
        ]
    ].sort_values(["trajectory_key", "controller", "nr_profile"])


def build_executive_summary(matrix_df: pd.DataFrame) -> pd.DataFrame:
    best_overall = (
        matrix_df.sort_values("Position_RMSE_m")
        .groupby(["trajectory_key", "trajectory_display"], as_index=False)
        .first()[
            [
                "trajectory_key",
                "trajectory_display",
                "comparison_label_final",
                "Position_RMSE_m",
                "Comp_Time_ms",
            ]
        ]
        .rename(
            columns={
                "comparison_label_final": "best_overall",
                "Position_RMSE_m": "best_overall_rmse_m",
                "Comp_Time_ms": "best_overall_comp_time_ms",
            }
        )
    )

    best_nr = (
        matrix_df[matrix_df["controller_key"].isin(set(NR_CONTROLLER_LABELS))]
        .sort_values("Position_RMSE_m")
        .groupby(["trajectory_key", "trajectory_display"], as_index=False)
        .first()[
            [
                "trajectory_key",
                "trajectory_display",
                "comparison_label_final",
                "Position_RMSE_m",
                "Comp_Time_ms",
            ]
        ]
        .rename(
            columns={
                "comparison_label_final": "best_nr_family",
                "Position_RMSE_m": "best_nr_rmse_m",
                "Comp_Time_ms": "best_nr_comp_time_ms",
            }
        )
    )

    best_nmpc = (
        matrix_df[matrix_df["controller_key"] == "nmpc"]
        .sort_values("Position_RMSE_m")
        .groupby(["trajectory_key", "trajectory_display"], as_index=False)
        .first()[
            [
                "trajectory_key",
                "trajectory_display",
                "comparison_label_final",
                "Position_RMSE_m",
                "Comp_Time_ms",
            ]
        ]
        .rename(
            columns={
                "comparison_label_final": "best_nmpc",
                "Position_RMSE_m": "best_nmpc_rmse_m",
                "Comp_Time_ms": "best_nmpc_comp_time_ms",
            }
        )
    )

    executive = best_overall.merge(
        best_nr,
        on=["trajectory_key", "trajectory_display"],
        how="left",
    ).merge(
        best_nmpc,
        on=["trajectory_key", "trajectory_display"],
        how="left",
    )
    executive["best_nr_minus_nmpc_m"] = executive["best_nr_rmse_m"] - executive["best_nmpc_rmse_m"]
    executive["best_nr_minus_nmpc_pct"] = 100.0 * executive["best_nr_minus_nmpc_m"] / executive["best_nmpc_rmse_m"]
    return executive[
        [
            "trajectory_key",
            "trajectory_display",
            "best_overall",
            "best_overall_rmse_m",
            "best_overall_comp_time_ms",
            "best_nr_family",
            "best_nr_rmse_m",
            "best_nmpc",
            "best_nmpc_rmse_m",
            "best_nr_minus_nmpc_m",
            "best_nr_minus_nmpc_pct",
        ]
    ].sort_values("trajectory_key")


def pivot_metric(
    matrix_df: pd.DataFrame,
    *,
    value_col: str,
    comparison_order: list[str],
) -> pd.DataFrame:
    pivot = matrix_df.pivot_table(
        index="trajectory_display",
        columns="comparison_label_final",
        values=value_col,
        aggfunc="first",
    )
    ordered_columns = [label for label in comparison_order if label in pivot.columns]
    return pivot.reindex(
        index=[TRAJECTORY_LABELS[key] for key in TRAJECTORIES],
        columns=ordered_columns,
    ).reset_index()


def cases() -> list[SweepCase]:
    return [
        SweepCase(
            name="nr_baseline_noff",
            controllers=("newton_raphson", "newton_raphson_enhanced", "nr_diff_flat"),
            nr_profile="baseline",
        ),
        SweepCase(
            name="nr_baseline_ff",
            controllers=("newton_raphson", "newton_raphson_enhanced", "nr_diff_flat"),
            nr_profile="baseline",
            ff=True,
        ),
        SweepCase(
            name="nr_workshop_noff",
            controllers=("newton_raphson", "newton_raphson_enhanced", "nr_diff_flat"),
            nr_profile="workshop",
        ),
        SweepCase(
            name="nr_workshop_ff",
            controllers=("newton_raphson", "newton_raphson_enhanced", "nr_diff_flat"),
            nr_profile="workshop",
            ff=True,
        ),
        SweepCase(name="nmpc_noff", controllers=("nmpc",)),
        SweepCase(name="nmpc_ff", controllers=("nmpc",), ff=True),
    ]


def main() -> None:
    args = parse_args()
    workspace_root = Path(args.workspace_root).resolve()
    px4_dir = Path(args.px4_dir).resolve()
    root = results_root(workspace_root, args.results_tag)
    aggregate_dir = root / "aggregate"
    aggregate_dir.mkdir(parents=True, exist_ok=True)

    matrix_df, run_records = run_matrix(
        workspace_root=workspace_root,
        px4_dir=px4_dir,
        root=root,
        reuse_existing=args.reuse_existing,
        cases=cases(),
    )

    if matrix_df.empty:
        raise RuntimeError("No result rows were collected.")

    (aggregate_dir / "run_records.json").write_text(json.dumps(run_records, indent=2, sort_keys=True) + "\n")
    matrix_df.to_csv(aggregate_dir / "raw_summary_rows.csv", index=False)

    matrix_df["comparison_label_final"] = matrix_df["comparison_label"]

    matrix_csv = aggregate_dir / "pre_fig8_main_matrix.csv"
    matrix_md = aggregate_dir / "pre_fig8_main_matrix.md"
    write_table(
        matrix_df[
            [
                "trajectory_key",
                "trajectory_display",
                "comparison_label_final",
                "Position_RMSE_m",
                "Comp_Time_ms",
                "results_dir",
            ]
        ].sort_values(["trajectory_key", "comparison_label_final"]),
        matrix_csv,
        matrix_md,
        float_cols=("Position_RMSE_m", "Comp_Time_ms"),
    )

    comparison_order = [
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

    position_table = pivot_metric(
        matrix_df,
        value_col="Position_RMSE_m",
        comparison_order=comparison_order,
    )
    write_table(
        position_table,
        aggregate_dir / "position_rmse_table.csv",
        aggregate_dir / "position_rmse_table.md",
        float_cols=tuple(column for column in position_table.columns if column != "trajectory_display"),
    )

    comp_table = pivot_metric(
        matrix_df,
        value_col="Comp_Time_ms",
        comparison_order=comparison_order,
    )
    write_table(
        comp_table,
        aggregate_dir / "comp_time_table.csv",
        aggregate_dir / "comp_time_table.md",
        float_cols=tuple(column for column in comp_table.columns if column != "trajectory_display"),
    )

    nr_improvements = build_nr_improvement_table(matrix_df)
    write_table(
        nr_improvements,
        aggregate_dir / "nr_workshop_improvements.csv",
        aggregate_dir / "nr_workshop_improvements.md",
        float_cols=("baseline", "workshop", "delta_m", "delta_pct"),
    )

    ff_effects = build_feedforward_effect_table(matrix_df)
    write_table(
        ff_effects,
        aggregate_dir / "feedforward_effects.csv",
        aggregate_dir / "feedforward_effects.md",
        float_cols=("noff", "ff", "delta_ff_minus_noff_m", "delta_ff_minus_noff_pct"),
    )

    best_rows = (
        matrix_df.sort_values("Position_RMSE_m")
        .groupby(["trajectory_key", "trajectory_display"], as_index=False)
        .first()[["trajectory_key", "trajectory_display", "comparison_label_final", "Position_RMSE_m", "Comp_Time_ms"]]
        .rename(columns={"comparison_label_final": "best_result"})
        .sort_values("trajectory_key")
    )
    write_table(
        best_rows,
        aggregate_dir / "best_by_trajectory.csv",
        aggregate_dir / "best_by_trajectory.md",
        float_cols=("Position_RMSE_m", "Comp_Time_ms"),
    )

    executive_summary = build_executive_summary(matrix_df)
    write_table(
        executive_summary,
        aggregate_dir / "executive_summary.csv",
        aggregate_dir / "executive_summary.md",
        float_cols=(
            "best_overall_rmse_m",
            "best_overall_comp_time_ms",
            "best_nr_rmse_m",
            "best_nmpc_rmse_m",
            "best_nr_minus_nmpc_m",
            "best_nr_minus_nmpc_pct",
        ),
    )

    manifest = {
        "results_root": str(root),
        "aggregate_dir": str(aggregate_dir),
        "trajectories": list(TRAJECTORIES),
        "artifacts": {
            "raw_summary_rows": str(aggregate_dir / "raw_summary_rows.csv"),
            "run_records": str(aggregate_dir / "run_records.json"),
            "main_matrix": str(matrix_csv),
            "position_rmse_table": str(aggregate_dir / "position_rmse_table.csv"),
            "comp_time_table": str(aggregate_dir / "comp_time_table.csv"),
            "nr_workshop_improvements": str(aggregate_dir / "nr_workshop_improvements.csv"),
            "feedforward_effects": str(aggregate_dir / "feedforward_effects.csv"),
            "best_by_trajectory": str(aggregate_dir / "best_by_trajectory.csv"),
            "executive_summary": str(aggregate_dir / "executive_summary.csv"),
        },
    }
    (aggregate_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    print(f"[pre-fig8-matrix] Complete. Aggregate results written to {aggregate_dir}")


if __name__ == "__main__":
    main()
