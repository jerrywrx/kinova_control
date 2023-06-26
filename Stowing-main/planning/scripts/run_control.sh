tool_type="gripper_sym_rod_robot_v2_surf_nocorr_full"
debug=1
target_shape_name="000"
optim_algo="GD"
CEM_sample_size=20
control_loss_type="mse"
subtarget=0
close_loop=1
planner_type='gnn'
max_n_actions=1

bash ./planning/scripts/control.sh $tool_type $debug $target_shape_name $optim_algo $CEM_sample_size $control_loss_type $subtarget $close_loop $planner_type $max_n_actions
