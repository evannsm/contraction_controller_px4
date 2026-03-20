1. I have updated the core.py of quad_trajectories
   1. Make the cpp version match it exactly even in the organization/order and naming scheme
   2. Update everything else accordingly in the controllers/etc if need be in light of the changes made 
2. Ensure that all controllers use only their respective quad_trajectories/_cpp depending on whether they're in cpp or python and have access to all the trajectories within them (the trajectories named after the contraction controller aren't exclusive to that controller they were just inspired by its development). 
3. Then once that is complete, ensure there is no dependence on quad_contraction_trajs and remove that directory
4. Git add, commit, and push to remote all changes in the submodules and in the monorepo that contains them




Ignore the comments below.
<!-- 1. `quad_trajectories` and `quad_trajectories_cpp` are the shared source of truth for trajectories across controller packages in this workspace.
1. Trajectories that overlap with the vetted workspace definitions keep the vetted names there, and contraction-specific variants live alongside them with `_contraction` suffixes so both can coexist.
2. `quad_contraction_trajs` remains only as a compatibility shim for legacy contraction imports; new controller and analysis code should import from `quad_trajectories` instead.
3. Shared support packages are intended to stay submodule-backed so this workspace can match the cross-compatible layout used in `/home/egmc/ws_px4_work/src`.
4. The expected local environment on this machine is `jazz`, then `cap` for the `px4_msgs` overlay, plus a compatible mamba env such as `ros2jazzy_px4_immrax` or `ros2jazzy_px4_acados`. -->


