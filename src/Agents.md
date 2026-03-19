1. `quad_trajectories` and `quad_trajectories_cpp` are the shared source of truth for trajectories across controller packages in this workspace.
2. Trajectories that overlap with the vetted workspace definitions keep the vetted names there, and contraction-specific variants live alongside them with `_contraction` suffixes so both can coexist.
3. `quad_contraction_trajs` remains only as a compatibility shim for legacy contraction imports; new controller and analysis code should import from `quad_trajectories` instead.
4. Shared support packages are intended to stay submodule-backed so this workspace can match the cross-compatible layout used in `/home/egmc/ws_px4_work/src`.
5. The expected local environment on this machine is `jazz`, then `cap` for the `px4_msgs` overlay, plus a compatible mamba env such as `ros2jazzy_px4_immrax` or `ros2jazzy_px4_acados`.
