# https://github.com/clnsmith/opensim-jam/blob/master/opensim-jam-release/examples/walking/README.md



runCOMAKIK = True
runCOMAK = True
runID = True
runJAM = True

import opensim as osim

from shutil import copytree
import os

osim.SimTK_PI * osim.SimTK_RADIAN_TO_DEGREE

cwd = os.getcwd()
print(cwd)

osim.Logger.setLevelString('Info')
# Off = 6
# Critical = 5
# Error = 4
# Warn = 3,
# Info = 2
# Debug = 1
# Trace = 

start_time = 1.26
end_time = 2.32

model_file = os.path.join(cwd, 'smith2019_noMeniscus.osim')
results_basename = 'walking'
ik_result_dir = os.path.join(cwd, './results/comak-inverse-kinematics')
comak_result_dir = os.path.join(cwd, './results/comak')
jnt_mech_result_dir = os.path.join(cwd, './results/joint-mechanics')
geometries = os.path.join(cwd, './Geometry')


os.makedirs('./results', exist_ok=True)
os.makedirs(ik_result_dir, exist_ok=True)
os.makedirs(comak_result_dir, exist_ok=True)
os.makedirs(jnt_mech_result_dir, exist_ok=True)

# model = osim.Model( model_file )
# for muscle in model.getMuscles():
#     MIF = muscle.getMaxIsometricForce()
#     muscle.setMaxIsometricForce( MIF*100)
#     # print( muscle.getName() )
# model.printToXML( './models/knee_healthy/lenhart2015/lenhart2015_strong.osim' )
# model_file = './models/knee_healthy/lenhart2015/lenhart2015_strong.osim'



# model = osim.Model( model_file )


# for c in model.getForceSet():
    # print(c.getName())

# for ligName in ['PT1','PT2','PT3','PT4','PT5','PT6']:
#     lig = model.getForceSet().get(ligName)
#     lig = osim.Blankevoort1991Ligament().safeDownCast(lig)
#     lig.set_slack_length( lig.get_slack_length() - 0.005 )

# obj = model.getForceSet().get('pf_contact') #.getConcreteClassName()
# obj = osim.Smith2018ArticularContactForce().safeDownCast(obj)
# obj.set_elastic_foundation_formulation('linear')
# obj.set_use_lumped_contact_model(True)
# obj.set_max_proximity( 0.01 )
# obj.set_min_proximity( 0 )


# obj = model.getForceSet().get('tf_contact') #.getConcreteClassName()
# obj = osim.Smith2018ArticularContactForce().safeDownCast(obj)
# obj.set_elastic_foundation_formulation('linear')
# obj.set_use_lumped_contact_model(True)
# obj.set_max_proximity( 0.02 )
# obj.set_min_proximity( 0 )

# model.printToXML( './models/knee_healthy/lenhart2015/lenhart2015_adjusted.osim' )
# model_file = './models/knee_healthy/lenhart2015/lenhart2015_adjusted.osim'

# When enabled (true), COMAK treats contact as lumped point contacts—a simplified model where contact forces are concentrated at discrete points (often centers of cartilage or key anatomical landmarks).
# When disabled (false), contact is modeled more realistically using distributed mesh-based representations, where forces are spread across the cartilage surface, accounting for more nuanced geometry.

# # %%
# Perform Inverse Kinematics
if runCOMAKIK:

    # copytree(geometries, os.path.join(ik_result_dir,'Geometry'), dirs_exist_ok=True )

    comak_ik = osim.COMAKInverseKinematicsTool()
    comak_ik.set_model_file(model_file)
    comak_ik.set_results_directory(ik_result_dir)
    comak_ik.set_results_prefix(results_basename)
    comak_ik.set_perform_secondary_constraint_sim(True)
    comak_ik.set_secondary_coupled_coordinate('/jointset/knee_r/knee_flex_r')
    comak_ik.set_secondary_coordinates(0,'/jointset/pf_r/pf_flex_r')
    comak_ik.set_secondary_coordinates(1,'/jointset/pf_r/pf_rot_r')
    comak_ik.set_secondary_coordinates(2,'/jointset/pf_r/pf_tilt_r')
    comak_ik.set_secondary_coordinates(3,'/jointset/pf_r/pf_tx_r')
    comak_ik.set_secondary_coordinates(4,'/jointset/pf_r/pf_ty_r')
    comak_ik.set_secondary_coordinates(5,'/jointset/pf_r/pf_tz_r')
    comak_ik.set_secondary_coordinates(6,'/jointset/knee_r/knee_add_r')
    comak_ik.set_secondary_coordinates(7,'/jointset/knee_r/knee_rot_r')
    comak_ik.set_secondary_coordinates(8,'/jointset/knee_r/knee_tx_r')
    comak_ik.set_secondary_coordinates(9,'/jointset/knee_r/knee_ty_r')
    comak_ik.set_secondary_coordinates(10,'/jointset/knee_r/knee_tz_r')

    comak_ik.set_secondary_constraint_sim_settle_threshold(1e-5) # def 1e-5
    comak_ik.set_secondary_constraint_sim_sweep_time(5)
    comak_ik.set_secondary_coupled_coordinate_start_value(-10) # from 0 deg knee flexion
    comak_ik.set_secondary_coupled_coordinate_stop_value(140) # to 100 deg knee flexion
    comak_ik.set_secondary_constraint_sim_integrator_accuracy(1e-4) # def 1e-6
    comak_ik.set_secondary_constraint_sim_internal_step_limit(-1) # def -1 (no limit)
    # if set_perform_secondary_constraint_sim is on, creates this file
    # if not, uses this file.
    comak_ik.set_secondary_constraint_function_file('./results/comak-inverse-kinematics/secondary_coordinate_constraint_functions.xml')
    comak_ik.set_constraint_function_num_interpolation_points(20)
    comak_ik.set_print_secondary_constraint_sim_results(False)
    comak_ik.set_constrained_model_file('./results/comak-inverse-kinematics/ik_constrained_model.osim')
    comak_ik.set_perform_inverse_kinematics(True)
    comak_ik.set_marker_file('./exp/overground_17.trc')
    comak_ik.set_output_motion_file('overground_17_ik.mot')
    comak_ik.set_time_range(0, 0)
    comak_ik.set_time_range(1, 100)
    comak_ik.set_report_errors(False)
    comak_ik.set_report_marker_locations(False)
    comak_ik.set_ik_constraint_weight( float('inf') ) # def inf
    # comak_ik.set_ik_constraint_weight( 100 ) # def inf
    comak_ik.set_ik_accuracy(1e-12)
    comak_ik.set_use_visualizer(False)
    # comak_ik.set_verbose(10)


    ik_task_set = osim.IKTaskSet()

    ik_task = osim.IKMarkerTask()

    # ik_task.setName('R_HJC')
    # ik_task.setWeight(0)
    # ik_task_set.cloneAndAppend(ik_task)

    # ik_task.setName('L_HJC')
    # ik_task.setWeight(0)
    # ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('S2')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.ASIS')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.PSIS')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.ASIS')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.PSIS')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.Clavicle')
    ik_task.setWeight(1)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.Clavicle')
    ik_task.setWeight(1)
    ik_task_set.cloneAndAppend(ik_task)

    # ik_task.setName('R.Scapula')
    # ik_task.setWeight(1)
    # ik_task_set.cloneAndAppend(ik_task)

    # ik_task.setName('L.Scapula')
    # ik_task.setWeight(1)
    # ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.Shoulder')
    ik_task.setWeight(1)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.Shoulder')
    ik_task.setWeight(1)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.Knee')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.TH1')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.TH2')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.TH3')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    # ik_task.setName('R.TH4')
    # ik_task.setWeight(5)
    # ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.Ankle')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.SH1')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.SH2')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.SH3')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.SH4')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.MT5')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('R.Heel')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.Knee')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.TH1')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.TH2')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.TH3')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.TH4')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.Ankle')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.SH1')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.SH2')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.SH3')
    ik_task.setWeight(5)
    ik_task_set.cloneAndAppend(ik_task)

    # ik_task.setName('L.SH4')
    # ik_task.setWeight(5)
    # ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.MT5')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    ik_task.setName('L.Heel')
    ik_task.setWeight(10)
    ik_task_set.cloneAndAppend(ik_task)

    comak_ik.set_IKTaskSet(ik_task_set)
    # comak_ik.printToXML('./inputs/comak_inverse_kinematics_settings.xml')

    print('Running COMAKInverseKinematicsTool...')
    comak_ik.run()














# %%
# Perform COMAK Simulation
if runCOMAK:

    comak = osim.COMAKTool()
    comak.set_model_file(model_file)
    comak.set_coordinates_file('./results/comak-inverse-kinematics/overground_17_ik.mot')
    comak.set_external_loads_file('./exp/overground_17_ext_loads.xml'),
    comak.set_results_directory(comak_result_dir)
    comak.set_results_prefix(results_basename)
    comak.set_replace_force_set(False)
    comak.set_force_set_file('./smith2019_reserve_actuators.xml')
    comak.set_start_time(start_time - 0.1)
    comak.set_stop_time(end_time + 0.1)
    comak.set_time_step(0.01)
    # comak.set_time_step(0.0125)
    comak.set_lowpass_filter_frequency(6)
    comak.set_print_processed_input_kinematics(False)

    # comak.set_prescribed_coordinates(0,'/jointset/gnd_pelvis/pelvis_tx')
    # comak.set_prescribed_coordinates(1,'/jointset/gnd_pelvis/pelvis_ty')
    # comak.set_prescribed_coordinates(2,'/jointset/gnd_pelvis/pelvis_tz')
    # comak.set_prescribed_coordinates(3,'/jointset/gnd_pelvis/pelvis_tilt')
    # comak.set_prescribed_coordinates(4,'/jointset/gnd_pelvis/pelvis_list')
    # comak.set_prescribed_coordinates(5,'/jointset/gnd_pelvis/pelvis_rot')
    # comak.set_prescribed_coordinates(6,'/jointset/mtp_r/mtp_angle_r')
    # comak.set_prescribed_coordinates(7,'/jointset/hip_l/hip_flex_l')
    # comak.set_prescribed_coordinates(8,'/jointset/hip_l/hip_add_l')
    # comak.set_prescribed_coordinates(9,'/jointset/hip_l/hip_rot_l')
    # comak.set_prescribed_coordinates(10,'/jointset/pf_l/pf_l_r3')
    # comak.set_prescribed_coordinates(11,'/jointset/pf_l/pf_l_tx')
    # comak.set_prescribed_coordinates(12,'/jointset/pf_l/pf_l_ty')
    # comak.set_prescribed_coordinates(13,'/jointset/knee_l/knee_flex_l')
    # comak.set_prescribed_coordinates(14,'/jointset/ankle_l/ankle_flex_l')
    # comak.set_prescribed_coordinates(15,'/jointset/subtalar_l/subt_angle_l')
    # comak.set_prescribed_coordinates(16,'/jointset/mtp_l/mtp_angle_l')
    # comak.set_prescribed_coordinates(17,'/jointset/pelvis_torso/lumbar_ext')
    # comak.set_prescribed_coordinates(18,'/jointset/pelvis_torso/lumbar_latbend')
    # comak.set_prescribed_coordinates(19,'/jointset/pelvis_torso/lumbar_rot')
    # comak.set_prescribed_coordinates(20,'/jointset/torso_neckhead/neck_ext')
    # comak.set_prescribed_coordinates(21,'/jointset/torso_neckhead/neck_latbend')
    # comak.set_prescribed_coordinates(22,'/jointset/torso_neckhead/neck_rot')
    # comak.set_prescribed_coordinates(23,'/jointset/acromial_r/arm_add_r')
    # comak.set_prescribed_coordinates(24,'/jointset/acromial_r/arm_flex_r')
    # comak.set_prescribed_coordinates(25,'/jointset/acromial_r/arm_rot_r')
    # comak.set_prescribed_coordinates(26,'/jointset/elbow_r/elbow_flex_r')
    # comak.set_prescribed_coordinates(27,'/jointset/radioulnar_r/pro_sup_r')
    # comak.set_prescribed_coordinates(28,'/jointset/radius_hand_r/wrist_flex_r')
    # comak.set_prescribed_coordinates(39,'/jointset/acromial_l/arm_add_l')
    # comak.set_prescribed_coordinates(30,'/jointset/acromial_l/arm_flex_l')
    # comak.set_prescribed_coordinates(31,'/jointset/acromial_l/arm_rot_l')
    # comak.set_prescribed_coordinates(32,'/jointset/elbow_l/elbow_flex_l')
    # comak.set_prescribed_coordinates(33,'/jointset/radioulnar_l/pro_sup_l')
    # comak.set_prescribed_coordinates(34,'/jointset/radius_hand_l/wrist_flex_l')
    # comak.set_prescribed_coordinates(23,'/jointset/subtalar_r/subt_angle_r')
    
    comak.set_primary_coordinates(0,'/jointset/hip_r/hip_flex_r')
    comak.set_primary_coordinates(1,'/jointset/hip_r/hip_add_r')
    comak.set_primary_coordinates(2,'/jointset/hip_r/hip_rot_r')
    comak.set_primary_coordinates(3,'/jointset/knee_r/knee_flex_r')
    comak.set_primary_coordinates(4,'/jointset/ankle_r/ankle_flex_r')
    # comak.set_primary_coordinates(5,'/jointset/subtalar_r/subt_angle_r')

    secondary_coord_set = osim.COMAKSecondaryCoordinateSet() 
    secondary_coord = osim.COMAKSecondaryCoordinate()

    secondary_coord.setName('knee_add_r')
    secondary_coord.set_max_change(0.005)
    secondary_coord.set_coordinate('/jointset/knee_r/knee_add_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('knee_rot_r')
    secondary_coord.set_max_change(0.005)
    secondary_coord.set_coordinate('/jointset/knee_r/knee_rot_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('knee_tx_r')
    secondary_coord.set_max_change(0.001)
    secondary_coord.set_coordinate('/jointset/knee_r/knee_tx_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('knee_ty_r')
    secondary_coord.set_max_change(0.001)
    secondary_coord.set_coordinate('/jointset/knee_r/knee_ty_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('knee_tz_r')
    secondary_coord.set_max_change(0.001)
    secondary_coord.set_coordinate('/jointset/knee_r/knee_tz_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('pf_flex_r')
    secondary_coord.set_max_change(0.005)
    secondary_coord.set_coordinate('/jointset/pf_r/pf_flex_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('pf_rot_r')
    secondary_coord.set_max_change(0.005)
    secondary_coord.set_coordinate('/jointset/pf_r/pf_rot_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('pf_tilt_r')
    secondary_coord.set_max_change(0.005)
    secondary_coord.set_coordinate('/jointset/pf_r/pf_tilt_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('pf_tx_r')
    secondary_coord.set_max_change(0.001)
    secondary_coord.set_coordinate('/jointset/pf_r/pf_tx_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('pf_ty_r')
    secondary_coord.set_max_change(0.001)
    secondary_coord.set_coordinate('/jointset/pf_r/pf_ty_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    secondary_coord.setName('pf_tz_r')
    secondary_coord.set_max_change(0.001)
    secondary_coord.set_coordinate('/jointset/pf_r/pf_tz_r')
    secondary_coord_set.cloneAndAppend(secondary_coord)

    comak.set_COMAKSecondaryCoordinateSet(secondary_coord_set)







    cost_fun_param_set = osim.COMAKCostFunctionParameterSet()
    cost_fun_param = osim.COMAKCostFunctionParameter()

    # 'set_activation_lower_bound'
    # 'set_activation_upper_bound'
    # 'set_actuator'
    # 'set_desired_activation'
    # 'set_weight'

    cost_fun_param.setName('gasmed_r')
    cost_fun_param.set_actuator('/forceset/gasmed_r')
    cost_fun_param.set_weight(osim.Constant(4))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('gaslat_r')
    cost_fun_param.set_actuator('/forceset/gaslat_r')
    cost_fun_param.set_weight(osim.Constant(7))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('soleus_r')
    cost_fun_param.set_actuator('/forceset/soleus_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('recfem_r')
    cost_fun_param.set_actuator('/forceset/recfem_r')
    cost_fun_param.set_weight(osim.Constant(3))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('glmed1_r')
    cost_fun_param.set_actuator('/forceset/glmed1_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('glmed2_r')
    cost_fun_param.set_actuator('/forceset/glmed1_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('glmed3_r')
    cost_fun_param.set_actuator('/forceset/glmed3_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('glmin1_r')
    cost_fun_param.set_actuator('/forceset/glmin1_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('glmin2_r')
    cost_fun_param.set_actuator('/forceset/glmin2_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('glmin3_r')
    cost_fun_param.set_actuator('/forceset/glmin3_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('bflh_r')
    cost_fun_param.set_actuator('/forceset/bflh_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('bfsh_r')
    cost_fun_param.set_actuator('/forceset/bfsh_r')
    cost_fun_param.set_weight(osim.Constant(0))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('semiten_r')
    cost_fun_param.set_actuator('/forceset/semiten_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    cost_fun_param.setName('semimem_r')
    cost_fun_param.set_actuator('/forceset/semimem_r')
    cost_fun_param.set_weight(osim.Constant(0.9))
    cost_fun_param_set.cloneAndAppend(cost_fun_param)

    comak.set_COMAKCostFunctionParameterSet(cost_fun_param_set)





    comak.set_settle_secondary_coordinates_at_start(True)
    comak.set_settle_threshold(1e-5) # def 1e-5
    comak.set_settle_accuracy(1e-5) # def 1e-6
    comak.set_settle_internal_step_limit(10000)
    comak.set_print_settle_sim_results(False)
    comak.set_settle_sim_results_directory(comak_result_dir)
    comak.set_settle_sim_results_prefix('walking_settle_sim')
    comak.set_max_iterations(200)
    comak.set_udot_tolerance(1)
    comak.set_udot_worse_case_tolerance(50)
    comak.set_unit_udot_epsilon(1e-8) # def 1e-8
    comak.set_optimization_scale_delta_coord(1)
    comak.set_ipopt_diagnostics_level(1)
    comak.set_ipopt_max_iterations(500)
    comak.set_ipopt_convergence_tolerance(1e-4)
    comak.set_ipopt_constraint_tolerance(1e-5)
    comak.set_ipopt_limited_memory_history(200)
    comak.set_ipopt_nlp_scaling_max_gradient(10000)
    comak.set_ipopt_nlp_scaling_min_value(1e-8)
    comak.set_ipopt_obj_scaling_factor(1)
    comak.set_use_muscle_physiology(False) # is not supported
    comak.set_activation_exponent(2)
    comak.set_contact_energy_weight(0)
    comak.set_non_muscle_actuator_weight(1000)
    comak.set_model_assembly_accuracy(1e-12)
    comak.set_use_visualizer(False)
    # comak.set_verbose(2)

    # comak.printToXML('./inputs/comak_settings.xml')

    print('Running COMAK Tool...')
    comak.run()



# perform inverse dynamics
if runID:
    coordinate_file = os.path.join(comak_result_dir,results_basename+'_values.sto')
    while not os.path.exists( coordinate_file ): pass
    indy = osim.InverseDynamicsTool()
    indy.setModelFileName(model_file)
    indy.setStartTime(start_time)
    indy.setEndTime(end_time)
    indy.setCoordinatesFileName( coordinate_file )
    indy.setExternalLoadsFileName( './exp/overground_17_ext_loads.xml' )
    indy.setLowpassCutoffFrequency(7)
    indy.setResultsDir( ik_result_dir )
    indy.setOutputGenForceFileName( f'{results_basename}_inverse_dynamics.sto')
    fexclude = osim.ArrayStr()
    fexclude.set(0, 'All')
    indy.setExcludedForces(fexclude)
    # indy.set_joints_to_report_body_forces('All')
    # indy.set_output_body_forces_file(join(WORKpath, 'output', f'{name}__body_forces_at_joints.sto'))
    # indy.printToXML(join(WORKpath, 'output', 'config', f'{name}_setup_ID.xml'))
    indy.run()





















# %% 
# Perform Joint Mechanics Analysis
if runJAM:
    jnt_mech = osim.JointMechanicsTool()
    jnt_mech.set_model_file(model_file)
    jnt_mech.set_input_states_file( os.path.join(comak_result_dir,results_basename+'_states.sto') )

    jnt_mech.set_use_activation_dynamics(False)
    jnt_mech.set_use_muscle_physiology(False)
    jnt_mech.set_use_tendon_compliance(False)

    jnt_mech.set_results_file_basename(results_basename)
    jnt_mech.set_results_directory(jnt_mech_result_dir)
    jnt_mech.set_start_time(start_time)
    jnt_mech.set_stop_time(end_time)
    jnt_mech.set_resample_step_size(-1)
    jnt_mech.set_normalize_to_cycle(True)
    jnt_mech.set_lowpass_filter_frequency(-1)
    jnt_mech.set_print_processed_kinematics(True)
    jnt_mech.set_contacts(0,'all')
    jnt_mech.set_contact_outputs(0,'all')
    jnt_mech.set_contact_mesh_properties(0,'none')
    jnt_mech.set_ligaments(0,'all')
    jnt_mech.set_ligament_outputs(0,'all')
    jnt_mech.set_muscles(0,'none')
    jnt_mech.set_muscle_outputs(0,'none')
    jnt_mech.set_attached_geometry_bodies(0,'none')

    jnt_mech.set_output_orientation_frame('/bodyset/patella_r') # patella_r
    jnt_mech.set_output_position_frame('/bodyset/patella_r')
    jnt_mech.set_write_vtp_files(False)
    jnt_mech.set_vtp_file_format('binary')
    jnt_mech.set_write_h5_file(False)
    jnt_mech.set_h5_kinematics_data(False)
    jnt_mech.set_h5_states_data(False)
    jnt_mech.set_write_transforms_file(False)
    jnt_mech.set_output_transforms_file_type('sto')
    jnt_mech.set_use_visualizer(False)
    # jnt_mech.set_verbose(0)

    analysis_set = osim.AnalysisSet()

    frc_reporter = osim.ForceReporter()
    frc_reporter.setName('ForceReporter')

    analysis_set.cloneAndAppend(frc_reporter)
    jnt_mech.set_AnalysisSet(analysis_set)
    # jnt_mech.printToXML('./inputs/joint_mechanics_settings.xml')

    print('Running JointMechanicsTool...')
    jnt_mech.run()




