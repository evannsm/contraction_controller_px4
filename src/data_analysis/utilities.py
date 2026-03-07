import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import re


# ============================================================================
# Enum Mappings: Logging.msg enums -> Human-readable names
# ============================================================================

PLATFORM_NAMES = {
    0: 'Simulation',
    1: 'Hardware'
}

CONTROLLER_NAMES = {
    0: 'NR Standard',
    1: 'NR Enhanced',
    2: 'MPC'
}

TRAJECTORY_NAMES = {
    0: 'Hover',
    1: 'Circle H',
    2: 'Circle V',
    3: 'Fig8 H',
    4: 'Fig8 VS',
    5: 'Fig8 VT',
    6: 'Triangle',
    7: 'Sawtooth',
    8: 'Helix'
}


# ============================================================================
# Data Loading and Preprocessing
# ============================================================================

def load_csv(file_path: str) -> pd.DataFrame:
    """
    Load a CSV file and clean column names by removing the '/plotjuggler/logging/' prefix if present.

    Parameters:
    -----------
    file_path : str
        Path to the CSV file

    Returns:
    --------
    pd.DataFrame
        DataFrame with cleaned column names
    """
    df = pd.read_csv(file_path)

    # Clean column names: remove '/plotjuggler/logging/' prefix if it exists
    df.columns = df.columns.str.replace('/plotjuggler/logging/', '', regex=False)
    df.columns = df.columns.str.replace('__time', 'time', regex=False)

    return df


def extract_metadata_from_data(df: pd.DataFrame) -> Dict[str, str]:
    """
    Extract platform, controller, and trajectory metadata from the data columns.
    This uses the actual enum values logged in the data instead of filename parsing.

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing platform, controller, and trajectory columns

    Returns:
    --------
    dict
        Dictionary with 'platform', 'controller', 'trajectory', 'traj_double', 'traj_spin' keys
    """
    metadata = {
        'platform': 'Unknown',
        'controller': 'Unknown',
        'trajectory': 'Unknown',
        'traj_double': False,
        'traj_spin': False
    }

    # String to standard name mappings
    PLATFORM_STR_MAP = {'sim': 'Simulation', 'hw': 'Hardware'}
    CONTROLLER_STR_MAP = {'nr': 'NR Standard', 'nr_enhanced': 'NR Enhanced', 'mpc': 'MPC'}
    TRAJECTORY_STR_MAP = {
        'circle_horz': 'Circle H',
        'circle_vert': 'Circle V',
        'fig8_horz': 'Fig8 H',
        'fig8_vert_short': 'Fig8 VS',
        'fig8_vert_tall': 'Fig8 VT',
        'sawtooth': 'Sawtooth',
        'triangle': 'Triangle',
        'helix': 'Helix'
    }

    # Extract platform
    if 'platform' in df.columns:
        platform_val = df['platform'].mode()[0]  # Most common value
        if isinstance(platform_val, str):
            metadata['platform'] = PLATFORM_STR_MAP.get(platform_val, platform_val)
        else:
            platform_val = int(platform_val)
            metadata['platform'] = PLATFORM_NAMES.get(platform_val, f'Platform {platform_val}')

    # Extract controller
    if 'controller' in df.columns:
        controller_val = df['controller'].mode()[0]  # Most common value
        if isinstance(controller_val, str):
            metadata['controller'] = CONTROLLER_STR_MAP.get(controller_val, controller_val)
        else:
            controller_val = int(controller_val)
            metadata['controller'] = CONTROLLER_NAMES.get(controller_val, f'Controller {controller_val}')

    # Extract trajectory
    if 'trajectory' in df.columns:
        trajectory_val = df['trajectory'].mode()[0]  # Most common value
        if isinstance(trajectory_val, str):
            metadata['trajectory'] = TRAJECTORY_STR_MAP.get(trajectory_val, trajectory_val)
        else:
            trajectory_val = int(trajectory_val)
            metadata['trajectory'] = TRAJECTORY_NAMES.get(trajectory_val, f'Trajectory {trajectory_val}')

    # Extract trajectory modifiers
    if 'traj_double' in df.columns:
        traj_double_val = df['traj_double'].mode()[0]
        if isinstance(traj_double_val, str):
            # Handle string format: "DblSpd", "NoSpd", etc.
            metadata['traj_double'] = 'dbl' in traj_double_val.lower() or '2x' in traj_double_val.lower()
        else:
            metadata['traj_double'] = bool(traj_double_val)

    if 'traj_spin' in df.columns:
        traj_spin_val = df['traj_spin'].mode()[0]
        if isinstance(traj_spin_val, str):
            # Handle string format: "Spin", "NoSpin", etc.
            metadata['traj_spin'] = 'spin' in traj_spin_val.lower() and 'no' not in traj_spin_val.lower()
        else:
            metadata['traj_spin'] = bool(traj_spin_val)

    return metadata


def extract_metadata_from_filename(filename: str) -> Dict[str, str]:
    """
    Extract controller type and trajectory type from filename.
    Expected format examples: 'controller_trajectory.csv', 'nr_circle.csv', etc.

    NOTE: This is a fallback method. Prefer extract_metadata_from_data() when possible.

    Parameters:
    -----------
    filename : str
        Filename to parse

    Returns:
    --------
    dict
        Dictionary with 'controller' and 'trajectory' keys
    """
    # Remove .csv extension
    name = Path(filename).stem

    # Try to split by underscore or other delimiters
    parts = re.split(r'[_\-.]', name)

    # For now, return the full name as both controller and trajectory
    # You can customize this based on your naming convention
    return {
        'controller': parts[0] if len(parts) > 0 else 'unknown',
        'trajectory': parts[1] if len(parts) > 1 else name,
        'filename': name
    }


def load_all_csvs(directory: str) -> Dict[str, pd.DataFrame]:
    """
    Load all CSV files from a directory.

    Parameters:
    -----------
    directory : str
        Path to directory containing CSV files

    Returns:
    --------
    dict
        Dictionary mapping filenames to DataFrames
    """
    csv_dir = Path(directory)
    data_dict = {}

    for csv_file in csv_dir.glob('*.csv'):
        data_dict[csv_file.name] = load_csv(str(csv_file))

    return data_dict


# ============================================================================
# Trajectory Analysis
# ============================================================================

def detect_trajectory_plane(df: pd.DataFrame, threshold: float = 0.1) -> str:
    """
    Automatically detect which plane the trajectory is moving in (XY, XZ, or YZ).

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing x, y, z columns
    threshold : float
        Minimum variance ratio to consider a dimension as "moving"

    Returns:
    --------
    str
        One of 'xy', 'xz', or 'yz' indicating the primary plane of motion
    """
    # Calculate variance in each reference dimension
    var_x = df['x_ref'].var() if 'x_ref' in df.columns else 0
    var_y = df['y_ref'].var() if 'y_ref' in df.columns else 0
    var_z = df['z_ref'].var() if 'z_ref' in df.columns else 0

    # Normalize variances
    total_var = var_x + var_y + var_z
    if total_var == 0:
        return 'xy'  # default

    var_x_norm = var_x / total_var
    var_y_norm = var_y / total_var
    var_z_norm = var_z / total_var

    # Determine which two dimensions have the most variance
    variances = {'x': var_x_norm, 'y': var_y_norm, 'z': var_z_norm}
    sorted_vars = sorted(variances.items(), key=lambda x: x[1], reverse=True)

    # Get the two most varying dimensions
    dim1, dim2 = sorted_vars[0][0], sorted_vars[1][0]

    # Return the plane in alphabetical order
    plane = ''.join(sorted([dim1, dim2]))
    return plane


def trim_trailing_and_leading_all_nan(X: np.ndarray) -> np.ndarray:
    """
    Remove trailing and leading rows that are all NaN.

    Parameters:
    -----------
    X : np.ndarray
        Input array

    Returns:
    --------
    np.ndarray
        Array with trailing all-NaN rows removed
    """
    if X.size == 0:
        return X
    keep = ~np.isnan(X).all(axis=1)
    if not keep.any():
        return X[:0]
    last_valid = np.max(np.where(keep))

    first_valid = np.min(np.where(keep))
    return X[first_valid:last_valid+1]



def align_reference_to_actual(df: pd.DataFrame, sampling_rate: float = 10.0) -> pd.DataFrame:
    """
    Align reference values to actual values by shifting reference backward in time
    to account for lookahead_time.

    The control system computes reference values lookahead_time seconds in the future.
    This function shifts the reference values backward to align them with actual values
    at the same point in the trajectory.

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing actual values, reference values, and lookahead_time
    sampling_rate : float
        Data sampling rate in Hz (default: 10.0 Hz)

    Returns:
    --------
    pd.DataFrame
        DataFrame with aligned reference values, trimmed to valid range
    """
    if 'lookahead_time' not in df.columns:
        print("Warning: 'lookahead_time' column not found. Returning original DataFrame.")
        return df

    # Get lookahead time (should be constant throughout)
    lookahead_time = df['lookahead_time'].iloc[0]

    # Calculate number of samples to shift
    shift_samples = int(round(lookahead_time * sampling_rate))

    if shift_samples == 0:
        return df

    print(f"Aligning data: shifting reference values back by {lookahead_time:.2f}s ({shift_samples} samples)")

    # Create a copy of the dataframe
    df_aligned = df.copy()

    # Get all reference columns (columns ending with '_ref')
    ref_columns = [col for col in df.columns if col.endswith('_ref')]

    # Shift reference columns backward in time
    for col in ref_columns:
        df_aligned[col] = df[col].shift(shift_samples)

    # Trim the dataframe to remove NaN values introduced by shifting
    # Remove the last shift_samples rows (which now have NaN reference values)
    df_aligned = df_aligned.iloc[:-shift_samples] if shift_samples > 0 else df_aligned

    return df_aligned


def get_flat_output_and_desired(df: pd.DataFrame, flip_z: bool = True, align_lookahead: bool = True, sampling_rate: float = 10.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract actual and reference trajectories from DataFrame.

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing position and reference columns
    flip_z : bool
        Whether to flip the z-axis (NED to ENU conversion)
    align_lookahead : bool
        Whether to align reference values to account for lookahead_time (default: True)
    sampling_rate : float
        Data sampling rate in Hz (default: 10.0 Hz), used for alignment

    Returns:
    --------
    tuple
        (actual_values, reference_values) as numpy arrays with shape (n, 4) for [x, y, z, yaw]
    """
    # Apply alignment if requested
    if align_lookahead:
        df = align_reference_to_actual(df, sampling_rate=sampling_rate)

    # Handle different column naming conventions
    angle_col = 'psi' if 'psi' in df.columns else 'yaw'
    angle_ref_col = 'psi_ref' if 'psi_ref' in df.columns else 'yaw_ref'

    actual_values = df[['x', 'y', 'z', angle_col]].to_numpy()
    reference_values = df[['x_ref', 'y_ref', 'z_ref', angle_ref_col]].to_numpy()

    actual_values_clean = trim_trailing_and_leading_all_nan(actual_values)
    reference_values_clean = trim_trailing_and_leading_all_nan(reference_values)

    n = min(len(actual_values_clean), len(reference_values_clean))
    actual_values_clean = actual_values_clean[:n]
    reference_values_clean = reference_values_clean[:n]

    # Flip z and z_ref if needed (NED to ENU conversion)
    if flip_z:
        actual_values_clean[:, 2] *= -1
        reference_values_clean[:, 2] *= -1

    # print(f"{actual_values_clean=}, {reference_values_clean=}")
    return actual_values_clean, reference_values_clean


# ============================================================================
# RMSE Calculation
# ============================================================================

def calculate_rmse(actual: np.ndarray, reference: np.ndarray) -> float:
    """
    Calculate Root Mean Squared Error between actual and reference trajectories.

    For position (x, y, z), uses direct error.
    For yaw (4th column if present), applies weight of 0.18.

    Parameters:
    -----------
    actual : np.ndarray
        Actual values with shape (n, 3) for [x,y,z] or (n, 4) for [x,y,z,yaw]
    reference : np.ndarray
        Reference values with same shape as actual

    Returns:
    --------
    float
        RMSE value
    """
    errors = actual - reference

    # If we have yaw component (4th column), apply weight of 0.18
    if errors.shape[1] >= 4:
        errors[:, 3] = 0.18 * errors[:, 3]

    squared_errors = errors ** 2
    mse = np.mean(np.sum(squared_errors, axis=1))
    return np.sqrt(mse)


def calculate_overall_rmse(df: pd.DataFrame, flip_z: bool = True, align_lookahead: bool = True, sampling_rate: float = 10.0) -> float:
    """
    Calculate the overall RMSE across x, y, z, and yaw compared to their reference values.

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing the actual values and reference values
    flip_z : bool
        Whether to flip the z-axis
    align_lookahead : bool
        Whether to align reference values to account for lookahead_time (default: True)
    sampling_rate : float
        Data sampling rate in Hz (default: 10.0 Hz), used for alignment

    Returns:
    --------
    float
        The overall RMSE across all dimensions
    """
    actual_values, reference_values = get_flat_output_and_desired(df, flip_z=flip_z, align_lookahead=align_lookahead, sampling_rate=sampling_rate)
    # print(f"{actual_values=}, {reference_values=}")

    return calculate_rmse(actual_values, reference_values)


def calculate_position_rmse(df: pd.DataFrame, flip_z: bool = True, align_lookahead: bool = True, sampling_rate: float = 10.0) -> float:
    """
    Calculate RMSE for position only (x, y, z).

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing position data
    flip_z : bool
        Whether to flip the z-axis
    align_lookahead : bool
        Whether to align reference values to account for lookahead_time (default: True)
    sampling_rate : float
        Data sampling rate in Hz (default: 10.0 Hz), used for alignment

    Returns:
    --------
    float
        Position RMSE
    """
    actual_values, reference_values = get_flat_output_and_desired(df, flip_z=flip_z, align_lookahead=align_lookahead, sampling_rate=sampling_rate)
    return calculate_rmse(actual_values[:, :3], reference_values[:, :3])


def calculate_rmse_per_axis(df: pd.DataFrame, flip_z: bool = True, align_lookahead: bool = True, sampling_rate: float = 10.0) -> Dict[str, float]:
    """
    Calculate RMSE for each axis separately.

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing trajectory data
    flip_z : bool
        Whether to flip the z-axis
    align_lookahead : bool
        Whether to align reference values to account for lookahead_time (default: True)
    sampling_rate : float
        Data sampling rate in Hz (default: 10.0 Hz), used for alignment

    Returns:
    --------
    dict
        Dictionary with RMSE values for each axis
    """
    actual_values, reference_values = get_flat_output_and_desired(df, flip_z=flip_z, align_lookahead=align_lookahead, sampling_rate=sampling_rate)

    return {
        'x': np.sqrt(np.mean((actual_values[:, 0] - reference_values[:, 0]) ** 2)),
        'y': np.sqrt(np.mean((actual_values[:, 1] - reference_values[:, 1]) ** 2)),
        'z': np.sqrt(np.mean((actual_values[:, 2] - reference_values[:, 2]) ** 2)),
        'yaw': np.sqrt(np.mean((actual_values[:, 3] - reference_values[:, 3]) ** 2)),
    }


def print_dataset_metadata(data_dict: Dict[str, pd.DataFrame]) -> None:
    """
    Print metadata information extracted from all datasets.

    Parameters:
    -----------
    data_dict : dict
        Dictionary mapping filenames to DataFrames
    """
    print("Dataset Metadata (from logged data):")
    print("=" * 80)

    for filename, df in data_dict.items():
        metadata = extract_metadata_from_data(df)

        print(f"\n{filename}:")
        print(f"  Platform:    {metadata['platform']}")
        print(f"  Controller:  {metadata['controller']}")
        print(f"  Trajectory:  {metadata['trajectory']}")

        modifiers = []
        if metadata['traj_double']:
            modifiers.append('2x speed')
        if metadata['traj_spin']:
            modifiers.append('spinning')

        if modifiers:
            print(f"  Modifiers:   {', '.join(modifiers)}")
        else:
            print(f"  Modifiers:   none")


def calculate_mean_comp_time(df: pd.DataFrame) -> Optional[float]:
    """
    Calculate mean computation time if the column exists.

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame potentially containing comp_time or ctrl_comp_time column

    Returns:
    --------
    float or None
        Mean computation time in milliseconds, or None if column doesn't exist
    """
    if 'comp_time' in df.columns:
        return df['comp_time'].mean() * 1000  # Convert to ms
    elif 'ctrl_comp_time' in df.columns:
        return df['ctrl_comp_time'].mean() * 1000  # Convert to ms
    return None


# ============================================================================
# Results Table Generation
# ============================================================================

def generate_results_table(data_dict: Dict[str, pd.DataFrame],
                          use_data_metadata: bool = True,
                          controller_map: Optional[Dict[str, str]] = None,
                          trajectory_map: Optional[Dict[str, str]] = None) -> pd.DataFrame:
    """
    Generate a results table with RMSE and computation time for all datasets.

    Parameters:
    -----------
    data_dict : dict
        Dictionary mapping filenames to DataFrames
    use_data_metadata : bool
        If True, extract metadata from data columns (platform, controller, trajectory).
        If False, fallback to filename parsing (deprecated).
    controller_map : dict, optional
        Mapping from filename patterns to controller names (only used if use_data_metadata=False)
    trajectory_map : dict, optional
        Mapping from filename patterns to trajectory names (only used if use_data_metadata=False)

    Returns:
    --------
    pd.DataFrame
        Results table with columns: Platform, Controller, Trajectory, Modifiers, Position_RMSE_m, Comp_Time_ms
    """
    results = []

    for filename, df in data_dict.items():
        if use_data_metadata:
            # Extract metadata from actual data columns (preferred method)
            metadata = extract_metadata_from_data(df)
            platform = metadata['platform']
            controller = metadata['controller']
            trajectory = metadata['trajectory']

            # Build trajectory modifier string
            modifiers = []
            if metadata['traj_double']:
                modifiers.append('2x')
            if metadata['traj_spin']:
                modifiers.append('spin')
            modifier_str = '+'.join(modifiers) if modifiers else '-'

        else:
            # Fallback to filename parsing (deprecated)
            metadata = extract_metadata_from_filename(filename)
            platform = 'Unknown'
            controller = metadata['controller']
            trajectory = metadata['trajectory']
            modifier_str = '-'

            # Apply custom mappings if provided
            if controller_map:
                for pattern, name in controller_map.items():
                    if pattern in filename:
                        controller = name
                        break

            if trajectory_map:
                for pattern, name in trajectory_map.items():
                    if pattern in filename:
                        trajectory = name
                        break

        rmse = calculate_position_rmse(df)
        comp_time = calculate_mean_comp_time(df)

        results.append({
            'Platform': platform,
            'Controller': controller,
            'Trajectory': trajectory,
            'Modifiers': modifier_str,
            'Position_RMSE_m': rmse,
            'Comp_Time_ms': comp_time if comp_time is not None else np.nan
        })

    results_df = pd.DataFrame(results)
    return results_df.sort_values(['Platform', 'Controller', 'Trajectory'])


def format_latex_table(df: pd.DataFrame) -> str:
    """
    Format a DataFrame as a LaTeX table suitable for papers.

    Parameters:
    -----------
    df : pd.DataFrame
        Results DataFrame

    Returns:
    --------
    str
        LaTeX table string
    """
    return df.to_latex(index=False, float_format="%.4f", escape=False)


# ============================================================================
# Plotting Functions
# ============================================================================

def setup_publication_style():
    """
    Set up matplotlib for publication-quality plots.
    """
    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "serif",
        "font.serif": ["Computer Modern Roman"],
        "font.size": 12,
        "axes.labelsize": 14,
        "axes.titlesize": 14,
        "legend.fontsize": 11,
        "xtick.labelsize": 11,
        "ytick.labelsize": 11,
        "figure.dpi": 100,
        "savefig.dpi": 300,
        "savefig.format": "pdf",
        "savefig.bbox": "tight",
        "lines.linewidth": 2,
        "lines.markersize": 6,
    })


def plot_trajectory_2d(ax, df: pd.DataFrame, plane: Optional[str] = None,
                       actual_color: str = 'red', ref_color: str = 'blue',
                       actual_label: str = 'Actual', ref_label: str = 'Reference',
                       flip_z: bool = True, align_lookahead: bool = True,
                       sampling_rate: float = 10.0, **kwargs):
    """
    Plot a 2D trajectory on given axes.

    Parameters:
    -----------
    ax : matplotlib.axes.Axes
        Axes to plot on
    df : pd.DataFrame
        DataFrame containing trajectory data
    plane : str, optional
        Which plane to plot ('xy', 'xz', or 'yz'). If None, auto-detect
    actual_color : str
        Color for actual trajectory
    ref_color : str
        Color for reference trajectory
    actual_label : str
        Label for actual trajectory
    ref_label : str
        Label for reference trajectory
    flip_z : bool
        Whether to flip the z-axis
    align_lookahead : bool
        Whether to align reference values to account for lookahead_time (default: True)
    sampling_rate : float
        Data sampling rate in Hz (default: 10.0 Hz), used for alignment
    **kwargs : dict
        Additional arguments passed to plot()
    """
    # Apply alignment if requested
    if align_lookahead:
        df = align_reference_to_actual(df, sampling_rate=sampling_rate)

    if plane is None:
        plane = detect_trajectory_plane(df)

    # Extract coordinates
    if plane == 'xy':
        x_col, y_col = 'x', 'y'
        x_ref_col, y_ref_col = 'x_ref', 'y_ref'
        xlabel, ylabel = r'$x$ [m]', r'$y$ [m]'
        z_flip = 1
    elif plane == 'xz':
        x_col, y_col = 'x', 'z'
        x_ref_col, y_ref_col = 'x_ref', 'z_ref'
        xlabel, ylabel = r'$x$ [m]', r'$z$ [m]'
        z_flip = -1 if flip_z else 1
    elif plane == 'yz':
        x_col, y_col = 'y', 'z'
        x_ref_col, y_ref_col = 'y_ref', 'z_ref'
        xlabel, ylabel = r'$y$ [m]', r'$z$ [m]'
        z_flip = -1 if flip_z else 1
    else:
        raise ValueError(f"Invalid plane: {plane}. Must be 'xy', 'xz', or 'yz'")

    # Apply z-flip if needed
    if y_col == 'z':
        y_data = df[y_col].values * z_flip
        y_ref_data = df[y_ref_col].values * z_flip
    else:
        y_data = df[y_col].values
        y_ref_data = df[y_ref_col].values

    # Plot
    ax.plot(df[x_col], y_data, color=actual_color, linestyle='-',
            label=actual_label, **kwargs)
    ax.plot(df[x_ref_col], y_ref_data, color=ref_color, linestyle='--',
            label=ref_label, **kwargs)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend()
    ax.grid(True, alpha=0.3)


def plot_multi_controller_comparison(data_dict: Dict[str, pd.DataFrame],
                                     controller_groups: Dict[str, List[str]],
                                     trajectory_order: Optional[List[str]] = None,
                                     figsize: Tuple[int, int] = (20, 12),
                                     save_path: Optional[str] = None):
    """
    Create a multi-row plot comparing different controllers across trajectories.
    Each row shows all trajectories for one controller.

    Parameters:
    -----------
    data_dict : dict
        Dictionary mapping filenames to DataFrames
    controller_groups : dict
        Dictionary mapping controller names to lists of filenames
        Example: {'NR': ['nr_circle.csv', 'nr_line.csv'], 'MPC': ['mpc_circle.csv', ...]}
    trajectory_order : list, optional
        Order of trajectories in columns. If None, use alphabetical
    figsize : tuple
        Figure size (width, height)
    save_path : str, optional
        Path to save the figure as PDF

    Returns:
    --------
    matplotlib.figure.Figure
        The created figure
    """
    setup_publication_style()

    # Determine number of controllers and trajectories
    n_controllers = len(controller_groups)

    # Get all unique trajectories
    all_trajectories = set()
    for files in controller_groups.values():
        for f in files:
            meta = extract_metadata_from_filename(f)
            all_trajectories.add(meta['trajectory'])

    if trajectory_order is None:
        trajectory_order = sorted(list(all_trajectories))

    n_trajectories = len(trajectory_order)

    # Create figure
    fig, axes = plt.subplots(n_controllers, n_trajectories,
                            figsize=figsize, squeeze=False)

    # Plot each controller-trajectory combination
    for i, (controller_name, file_list) in enumerate(controller_groups.items()):
        for j, traj_name in enumerate(trajectory_order):
            ax = axes[i, j]

            # Find the matching file
            matching_file = None
            for filename in file_list:
                meta = extract_metadata_from_filename(filename)
                if meta['trajectory'] == traj_name:
                    matching_file = filename
                    break

            if matching_file and matching_file in data_dict:
                df = data_dict[matching_file]
                plot_trajectory_2d(ax, df, actual_label='Actual', ref_label='Ref')

                # Add title with RMSE
                rmse = calculate_position_rmse(df)
                ax.set_title(f"{controller_name}: {traj_name}\nRMSE: {rmse:.3f}m",
                           fontsize=12, fontweight='bold')
            else:
                ax.text(0.5, 0.5, 'No data', ha='center', va='center',
                       transform=ax.transAxes)
                ax.set_title(f"{controller_name}: {traj_name}", fontsize=12)

            # Remove y-label for non-leftmost plots
            if j > 0:
                ax.set_ylabel('')

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, format='pdf', bbox_inches='tight')
        print(f"Figure saved to {save_path}")

    return fig


def plot_time_series(df: pd.DataFrame,
                     vars_to_plot: List[str] = ['x', 'y', 'z', 'yaw'],
                     figsize: Tuple[int, int] = (12, 8),
                     flip_z: bool = True,
                     align_lookahead: bool = True,
                     sampling_rate: float = 10.0,
                     save_path: Optional[str] = None):
    """
    Plot time series of variables with their references.

    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing trajectory data
    vars_to_plot : list
        List of variables to plot
    figsize : tuple
        Figure size
    flip_z : bool
        Whether to flip the z-axis
    align_lookahead : bool
        Whether to align reference values to account for lookahead_time (default: True)
    sampling_rate : float
        Data sampling rate in Hz (default: 10.0 Hz), used for alignment
    save_path : str, optional
        Path to save the figure

    Returns:
    --------
    matplotlib.figure.Figure
        The created figure
    """
    setup_publication_style()

    # Apply alignment if requested
    if align_lookahead:
        df = align_reference_to_actual(df, sampling_rate=sampling_rate)

    n_vars = len(vars_to_plot)
    fig, axes = plt.subplots(n_vars, 1, figsize=figsize, sharex=True)

    if n_vars == 1:
        axes = [axes]

    # Use traj_time instead of time/timestamp
    time = df['traj_time'].values if 'traj_time' in df.columns else (df['time'].values if 'time' in df.columns else np.arange(len(df)))

    for i, var in enumerate(vars_to_plot):
        ax = axes[i]

        # Get data
        var_data = df[var].values
        var_ref_data = df[f'{var}_ref'].values if f'{var}_ref' in df.columns else None

        # Apply z-flip if needed
        if var == 'z' and flip_z:
            var_data = var_data * -1
            if var_ref_data is not None:
                var_ref_data = var_ref_data * -1

        # Plot
        ax.plot(time, var_data, 'r-', label=f'${var}$', linewidth=1.5)
        if var_ref_data is not None:
            ax.plot(time, var_ref_data, 'b--', label=f'${var}_{{ref}}$', linewidth=1.5)

        # Labels
        unit = '[m]' if var in ['x', 'y', 'z'] else '[rad]'
        ax.set_ylabel(f'${var}$ {unit}')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, format='pdf', bbox_inches='tight')
        print(f"Figure saved to {save_path}")

    return fig
