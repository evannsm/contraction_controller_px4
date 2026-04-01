#!/usr/bin/env python3
"""Run the remaining Agents.md comparison matrix and aggregate the results."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

import pandas as pd


TRAJECTORIES = (
    "hover_contraction",
    "fig8_contraction",
    "fig8_heading_contraction",
    "spiral_contraction",
    "trefoil_contraction",
)

TRAJECTORY_LABELS = {
    "hover_contraction": "Hover Contraction",
    "fig8_contraction": "Fig8 Contraction",
    "fig8_heading_contraction": "Fig8 Heading Contraction",
    "spiral_contraction": "Spiral Contraction",
    "trefoil_contraction": "Trefoil Contraction",
}

NR_CONTROLLER_LABELS = {
    "newton_raphson": "Standard NR",
    "newton_raphson_enhanced": "NR Enhanced",
    "nr_diff_flat": "NR Diff-Flat",
    "newton_raphson_cpp": "Standard NR C++",
    "newton_raphson_enhanced_cpp": "NR Enhanced C++",
    "nr_diff_flat_cpp": "NR Diff-Flat C++",
}

SUMMARY_CONTROLLER_TO_KEY = {
    "NR Standard": "newton_raphson",
    "NR Enhanced": "newton_raphson_enhanced",
    "NR Diff-Flat": "nr_diff_flat",
    "NR Diff-Flat (JAX)": "nr_diff_flat",
    "NR Diff-Flat (NumPy)": "nr_diff_flat",
    "NMPC": "nmpc",
    "MPC": "nmpc",
    "Contraction": "contraction",
    "Flatness Feedforward": "ff_f8",
    "FBL": "fbl",
}

REQUESTED_CONTROLLER_TO_LABEL = {
    "contraction": "contraction",
    "nmpc": "NMPC",
    "nmpc_cpp": "NMPC",
    "ff_f8": "Flatness Feedforward",
    "fbl": "FBL",
    "newton_raphson": "NR Standard",
    "newton_raphson_enhanced": "NR Enhanced",
    "nr_diff_flat": "NR Diff-Flat",
    "newton_raphson_cpp": "NR Standard",
    "newton_raphson_enhanced_cpp": "NR Enhanced",
    "nr_diff_flat_cpp": "NR Diff-Flat",
}

FBL_PROFILES = {
    "default": {},
    "balanced": {
        "kp_xy": 0.16,
        "kv_xy": 0.22,
        "kp_z": 0.42,
        "kv_z": 0.32,
        "kp_att": 1.45,
        "kp_yaw": 1.75,
        "kd_body_rates": 0.22,
        "max_tilt_cmd": 0.40,
    },
    "aggressive": {
        "kp_xy": 0.20,
        "kv_xy": 0.28,
        "kp_z": 0.52,
        "kv_z": 0.38,
        "kp_att": 1.65,
        "kp_yaw": 2.00,
        "kd_body_rates": 0.24,
        "max_tilt_cmd": 0.45,
    },
}


@dataclass(frozen=True)
class SweepCase:
    name: str
    controllers: tuple[str, ...]
    nr_profile: str = "baseline"
    ff: bool = False
    no_feedforward: bool = False
    p_feedback: bool = False
    ramp_seconds: float | None = None
    fbl_gain_profile: str | None = None
    fbl_gains: dict[str, float] = field(default_factory=dict)
    allow_failure: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the full Agents.md comparison matrix and aggregate the outputs."
    )
    parser.add_argument(
        "--workspace-root",
        default=str(Path(__file__).resolve().parents[2]),
        help="Workspace root. Defaults to the current repository root.",
    )
    parser.add_argument(
        "--results-tag",
        default=f"agents_matrix_{datetime.now().strftime('%Y%m%d')}",
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
    parser.add_argument(
        "--skip-main-matrix",
        action="store_true",
        help="Skip the Python controller comparison matrix.",
    )
    parser.add_argument(
        "--skip-fbl-tuning",
        action="store_true",
        help="Skip the FBL gain sweep.",
    )
    parser.add_argument(
        "--skip-cpp-acceptance",
        action="store_true",
        help="Skip the C++ NR-family acceptance runs.",
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


def add_fly_flag(cmd: list[str], flag: str, value: Any) -> None:
    if value is None:
        return
    cmd.extend([flag, str(value)])


def run_case(
    *,
    workspace_root: Path,
    px4_dir: Path,
    results_dir: Path,
    trajectory: str,
    phase: str,
    case: SweepCase,
    reuse_existing: bool,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    summary_path = results_dir / "analysis" / "summary.csv"
    record: dict[str, Any] = {
        "phase": phase,
        "trajectory_key": trajectory,
        "trajectory_label": TRAJECTORY_LABELS[trajectory],
        "case_name": case.name,
        "controllers": list(case.controllers),
        "nr_profile_requested": case.nr_profile,
        "ff_requested": case.ff,
        "no_feedforward_requested": case.no_feedforward,
        "p_feedback_requested": case.p_feedback,
        "fbl_gain_profile": case.fbl_gain_profile or "",
        "fbl_gains": case.fbl_gains,
        "results_dir": str(results_dir),
        "summary_csv": str(summary_path),
    }

    if reuse_existing and summary_path.is_file():
        record["status"] = "reused"
        df = pd.read_csv(summary_path)
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

    if trajectory == "hover_contraction":
        cmd.extend(["--hover-mode", "1"])
    if case.nr_profile:
        cmd.extend(["--nr-profile", case.nr_profile])
    if case.ff:
        cmd.append("--ff")
    if case.no_feedforward:
        cmd.append("--no-feedforward")
    if case.p_feedback:
        cmd.append("--p-feedback")
    add_fly_flag(cmd, "--ramp-seconds", case.ramp_seconds)

    for key, value in case.fbl_gains.items():
        add_fly_flag(cmd, f"--{key.replace('_', '-')}", value)

    print(
        f"[agents-matrix] {phase}: {trajectory} :: {case.name} "
        f"({', '.join(case.controllers)})"
    )
    result = subprocess.run(cmd, cwd=workspace_root)
    if result.returncode != 0:
        record["status"] = "failed"
        if case.allow_failure:
            return pd.DataFrame(), record
        raise RuntimeError(f"Command failed for {phase}/{trajectory}/{case.name}")

    if not summary_path.is_file():
        raise FileNotFoundError(f"Expected analysis summary was not created: {summary_path}")

    record["status"] = "completed"
    df = pd.read_csv(summary_path)
    return df, record


def modifiers_set(modifier_text: str) -> set[str]:
    if not isinstance(modifier_text, str) or modifier_text == "-":
        return set()
    return {token for token in modifier_text.split("+") if token}


def normalize_controller_key(controller_label: str) -> str:
    return SUMMARY_CONTROLLER_TO_KEY.get(controller_label, controller_label.lower().replace(" ", "_"))


def comparison_label(row: pd.Series) -> str:
    controller_key = row["controller_key"]
    modifiers = modifiers_set(row["Modifiers"])

    if controller_key in {"newton_raphson", "newton_raphson_enhanced", "nr_diff_flat"}:
        family = NR_CONTROLLER_LABELS[controller_key]
        nr_profile = "workshop" if "workshop" in modifiers else "baseline"
        ff_mode = "ff" if "ff" in modifiers else "noff"
        return f"{family} {nr_profile} {ff_mode}"

    if controller_key == "nmpc":
        ff_mode = "ff" if "ff" in modifiers else "noff"
        return f"NMPC {ff_mode}"

    if controller_key == "contraction":
        ff_mode = "noff" if row["no_feedforward_requested"] else "ff"
        return f"Contraction {ff_mode}"

    if controller_key == "ff_f8":
        return "Flatness Feedforward"

    if controller_key == "fbl":
        profile = row["fbl_gain_profile"] or "default"
        return f"FBL {profile}"

    if controller_key in {"newton_raphson_cpp", "newton_raphson_enhanced_cpp", "nr_diff_flat_cpp"}:
        family = NR_CONTROLLER_LABELS[controller_key]
        nr_profile = "workshop" if "workshop" in modifiers else "baseline"
        return f"{family} {nr_profile}"

    return str(row["Controller"])


def augment_rows(df: pd.DataFrame, record: dict[str, Any]) -> pd.DataFrame:
    if df.empty:
        return df

    augmented = df.copy()
    for key, value in record.items():
        if key == "fbl_gains":
            augmented[key] = json.dumps(value, sort_keys=True)
        else:
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


def summarize_fbl_tuning(raw_df: pd.DataFrame, aggregate_dir: Path) -> str:
    tuning_df = raw_df[(raw_df["phase"] == "fbl_tuning") & (raw_df["controller_key"] == "fbl")].copy()
    if tuning_df.empty:
        return "default"

    summary = (
        tuning_df.groupby("fbl_gain_profile", as_index=False)
        .agg(
            trajectories_completed=("trajectory_key", "nunique"),
            mean_position_rmse_m=("Position_RMSE_m", "mean"),
            mean_comp_time_ms=("Comp_Time_ms", "mean"),
            best_single_run_rmse_m=("Position_RMSE_m", "min"),
        )
        .sort_values(["mean_position_rmse_m", "mean_comp_time_ms", "fbl_gain_profile"])
    )
    summary["complete_profile"] = summary["trajectories_completed"] == len(TRAJECTORIES)
    write_table(
        summary,
        aggregate_dir / "fbl_tuning_summary.csv",
        aggregate_dir / "fbl_tuning_summary.md",
        float_cols=("mean_position_rmse_m", "mean_comp_time_ms", "best_single_run_rmse_m"),
    )

    complete = summary[summary["complete_profile"]]
    if not complete.empty:
        return str(complete.iloc[0]["fbl_gain_profile"])
    return "default"


def build_final_matrix(raw_df: pd.DataFrame, best_fbl_profile: str) -> pd.DataFrame:
    matrix_df = raw_df[raw_df["phase"].isin({"main_matrix", "fbl_tuning"})].copy()
    matrix_df = matrix_df[
        ~(
            (matrix_df["controller_key"] == "fbl")
            & ~matrix_df["fbl_gain_profile"].isin(["default", best_fbl_profile])
        )
    ]
    matrix_df = matrix_df[
        ~(
            (matrix_df["phase"] == "fbl_tuning")
            & (matrix_df["controller_key"] != "fbl")
        )
    ]

    def final_label(row: pd.Series) -> str:
        if row["controller_key"] == "fbl":
            if row["fbl_gain_profile"] == "default":
                return "FBL default"
            return f"FBL tuned ({best_fbl_profile})"
        return row["comparison_label"]

    matrix_df["comparison_label_final"] = matrix_df.apply(final_label, axis=1)
    matrix_df["trajectory_display"] = matrix_df["trajectory_key"].map(TRAJECTORY_LABELS)
    return matrix_df


def build_nr_improvement_table(matrix_df: pd.DataFrame) -> pd.DataFrame:
    nr_df = matrix_df[matrix_df["controller_key"].isin({"newton_raphson", "newton_raphson_enhanced", "nr_diff_flat"})].copy()
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

    for controller_key in ("newton_raphson", "newton_raphson_enhanced", "nr_diff_flat"):
        subset = matrix_df[matrix_df["controller_key"] == controller_key].copy()
        subset["nr_profile"] = subset["Modifiers"].map(
            lambda mod: "workshop" if "workshop" in modifiers_set(mod) else "baseline"
        )
        subset["ff_mode"] = subset["Modifiers"].map(lambda mod: "ff" if "ff" in modifiers_set(mod) else "noff")
        pivot = subset.pivot_table(
            index=["trajectory_key", "trajectory_display", "nr_profile"],
            columns="ff_mode",
            values="Position_RMSE_m",
            aggfunc="first",
        ).reset_index()
        if "ff" not in pivot.columns or "noff" not in pivot.columns:
            continue
        pivot["controller"] = NR_CONTROLLER_LABELS[controller_key]
        pivot["delta_ff_minus_noff_m"] = pivot["ff"] - pivot["noff"]
        pivot["delta_ff_minus_noff_pct"] = 100.0 * pivot["delta_ff_minus_noff_m"] / pivot["noff"]
        ff_rows.extend(pivot.to_dict("records"))

    for controller_key, label in (("nmpc", "NMPC"), ("contraction", "Contraction")):
        subset = matrix_df[matrix_df["controller_key"] == controller_key].copy()
        if subset.empty:
            continue
        if controller_key == "contraction":
            subset["ff_mode"] = subset["no_feedforward_requested"].map(lambda disabled: "noff" if disabled else "ff")
        else:
            subset["ff_mode"] = subset["Modifiers"].map(lambda mod: "ff" if "ff" in modifiers_set(mod) else "noff")
        pivot = subset.pivot_table(
            index=["trajectory_key", "trajectory_display"],
            columns="ff_mode",
            values="Position_RMSE_m",
            aggfunc="first",
        ).reset_index()
        if "ff" not in pivot.columns or "noff" not in pivot.columns:
            continue
        pivot["controller"] = label
        pivot["nr_profile"] = "-"
        pivot["delta_ff_minus_noff_m"] = pivot["ff"] - pivot["noff"]
        pivot["delta_ff_minus_noff_pct"] = 100.0 * pivot["delta_ff_minus_noff_m"] / pivot["noff"]
        ff_rows.extend(pivot.to_dict("records"))

    if not ff_rows:
        return pd.DataFrame()

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
    return pivot.reindex(index=[TRAJECTORY_LABELS[key] for key in TRAJECTORIES], columns=ordered_columns).reset_index()


def main_matrix_cases() -> list[SweepCase]:
    return [
        SweepCase(name="contraction_ff", controllers=("contraction",)),
        SweepCase(name="contraction_noff", controllers=("contraction",), no_feedforward=True),
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
        SweepCase(name="flat_ff", controllers=("ff_f8",)),
    ]


def cpp_acceptance_cases() -> list[SweepCase]:
    return [
        SweepCase(
            name="cpp_nr_family_baseline",
            controllers=("newton_raphson_cpp", "newton_raphson_enhanced_cpp", "nr_diff_flat_cpp"),
            nr_profile="baseline",
        ),
        SweepCase(
            name="cpp_nr_family_workshop",
            controllers=("newton_raphson_cpp", "newton_raphson_enhanced_cpp", "nr_diff_flat_cpp"),
            nr_profile="workshop",
        ),
    ]


def run_phase(
    *,
    workspace_root: Path,
    px4_dir: Path,
    root: Path,
    phase: str,
    cases: list[SweepCase],
    reuse_existing: bool,
    trajectories: tuple[str, ...] = TRAJECTORIES,
) -> tuple[pd.DataFrame, list[dict[str, Any]]]:
    rows: list[pd.DataFrame] = []
    records: list[dict[str, Any]] = []

    for trajectory in trajectories:
        for case in cases:
            case_dir = root / phase / trajectory / case.name
            df, record = run_case(
                workspace_root=workspace_root,
                px4_dir=px4_dir,
                results_dir=case_dir,
                trajectory=trajectory,
                phase=phase,
                case=case,
                reuse_existing=reuse_existing,
            )
            records.append(record)
            if not df.empty:
                rows.append(augment_rows(df, record))

    combined = pd.concat(rows, ignore_index=True) if rows else pd.DataFrame()
    return combined, records


def run_fbl_tuning_phase(
    *,
    workspace_root: Path,
    px4_dir: Path,
    root: Path,
    reuse_existing: bool,
) -> tuple[pd.DataFrame, list[dict[str, Any]]]:
    cases = [
        SweepCase(
            name=f"fbl_{profile_name}",
            controllers=("fbl",),
            p_feedback=True,
            fbl_gain_profile=profile_name,
            fbl_gains=gains,
            allow_failure=True,
        )
        for profile_name, gains in FBL_PROFILES.items()
    ]
    return run_phase(
        workspace_root=workspace_root,
        px4_dir=px4_dir,
        root=root,
        phase="fbl_tuning",
        cases=cases,
        reuse_existing=reuse_existing,
    )


def main() -> None:
    args = parse_args()
    workspace_root = Path(args.workspace_root).resolve()
    px4_dir = Path(args.px4_dir).resolve()
    root = results_root(workspace_root, args.results_tag)
    aggregate_dir = root / "aggregate"
    aggregate_dir.mkdir(parents=True, exist_ok=True)

    all_rows: list[pd.DataFrame] = []
    run_records: list[dict[str, Any]] = []

    if not args.skip_fbl_tuning:
        tuning_df, tuning_records = run_fbl_tuning_phase(
            workspace_root=workspace_root,
            px4_dir=px4_dir,
            root=root,
            reuse_existing=args.reuse_existing,
        )
        all_rows.append(tuning_df)
        run_records.extend(tuning_records)

    best_fbl_profile = "default"
    raw_df = pd.concat(all_rows, ignore_index=True) if all_rows else pd.DataFrame()
    if not raw_df.empty:
        best_fbl_profile = summarize_fbl_tuning(raw_df, aggregate_dir)

    if not args.skip_main_matrix:
        matrix_df, matrix_records = run_phase(
            workspace_root=workspace_root,
            px4_dir=px4_dir,
            root=root,
            phase="main_matrix",
            cases=main_matrix_cases(),
            reuse_existing=args.reuse_existing,
        )
        all_rows.append(matrix_df)
        run_records.extend(matrix_records)

    if not args.skip_cpp_acceptance:
        cpp_df, cpp_records = run_phase(
            workspace_root=workspace_root,
            px4_dir=px4_dir,
            root=root,
            phase="cpp_acceptance",
            cases=cpp_acceptance_cases(),
            reuse_existing=args.reuse_existing,
            trajectories=("fig8_contraction",),
        )
        all_rows.append(cpp_df)
        run_records.extend(cpp_records)

    raw_df = pd.concat([df for df in all_rows if not df.empty], ignore_index=True) if all_rows else pd.DataFrame()
    if raw_df.empty:
        raise RuntimeError("No result rows were collected.")

    (aggregate_dir / "run_records.json").write_text(json.dumps(run_records, indent=2, sort_keys=True) + "\n")
    raw_df.to_csv(aggregate_dir / "raw_summary_rows.csv", index=False)

    best_fbl_profile = summarize_fbl_tuning(raw_df, aggregate_dir)

    matrix_df = build_final_matrix(raw_df, best_fbl_profile)
    matrix_csv = aggregate_dir / "agents_main_matrix.csv"
    matrix_md = aggregate_dir / "agents_main_matrix.md"
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
        "Contraction ff",
        "Contraction noff",
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
        "Flatness Feedforward",
        "FBL default",
        f"FBL tuned ({best_fbl_profile})",
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
    if not nr_improvements.empty:
        write_table(
            nr_improvements,
            aggregate_dir / "nr_workshop_improvements.csv",
            aggregate_dir / "nr_workshop_improvements.md",
            float_cols=("baseline", "workshop", "delta_m", "delta_pct"),
        )

    ff_effects = build_feedforward_effect_table(matrix_df)
    if not ff_effects.empty:
        write_table(
            ff_effects,
            aggregate_dir / "feedforward_effects.csv",
            aggregate_dir / "feedforward_effects.md",
            float_cols=("noff", "ff", "delta_ff_minus_noff_m", "delta_ff_minus_noff_pct"),
        )

    cpp_summary = raw_df[raw_df["phase"] == "cpp_acceptance"][
        [
            "trajectory_key",
            "trajectory_display",
            "comparison_label",
            "Position_RMSE_m",
            "Comp_Time_ms",
            "results_dir",
        ]
    ].sort_values(["trajectory_key", "comparison_label"])
    if not cpp_summary.empty:
        write_table(
            cpp_summary,
            aggregate_dir / "cpp_acceptance_summary.csv",
            aggregate_dir / "cpp_acceptance_summary.md",
            float_cols=("Position_RMSE_m", "Comp_Time_ms"),
        )

    final_manifest = {
        "results_root": str(root),
        "aggregate_dir": str(aggregate_dir),
        "best_fbl_profile": best_fbl_profile,
        "best_fbl_gains": FBL_PROFILES.get(best_fbl_profile, {}),
        "artifacts": {
            "raw_summary_rows": str(aggregate_dir / "raw_summary_rows.csv"),
            "run_records": str(aggregate_dir / "run_records.json"),
            "main_matrix": str(matrix_csv),
            "position_rmse_table": str(aggregate_dir / "position_rmse_table.csv"),
            "comp_time_table": str(aggregate_dir / "comp_time_table.csv"),
            "fbl_tuning_summary": str(aggregate_dir / "fbl_tuning_summary.csv"),
            "nr_workshop_improvements": str(aggregate_dir / "nr_workshop_improvements.csv"),
            "feedforward_effects": str(aggregate_dir / "feedforward_effects.csv"),
            "cpp_acceptance_summary": str(aggregate_dir / "cpp_acceptance_summary.csv"),
        },
    }
    (aggregate_dir / "manifest.json").write_text(json.dumps(final_manifest, indent=2, sort_keys=True) + "\n")

    print(f"[agents-matrix] Complete. Aggregate results written to {aggregate_dir}")
    print(f"[agents-matrix] Best FBL profile: {best_fbl_profile}")


if __name__ == "__main__":
    main()
