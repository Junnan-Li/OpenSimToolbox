



clear all
close all
clc

import org.opensim.modeling.*

folder_path = pwd;
geometry_folder_path = strcat(folder_path,'\geometry_folder\Geometry_MoBL_ARMS\');
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);

IK_on = 0;

%% load model

model = osim_model(strcat(folder_path,'\model\Hand_Wrist_Model_for_development.osim'));

% check body list, joint list, ...
model.BodySet_list;
model.JointSet_list;
model.CoordinateSet_list;
model.MuscleSet_list;

% delete markers
model.delete_all_markers;
%
% enable visualization
% model.set_visualize;

%%
body_name = {'2distph'}; % name of the attached body
pos_vec = {Vec3(0,-0.1,0)}; % relative position in corresponding body frame
model.add_marker_points('index_tip', body_name,pos_vec);

%% set the list of the target coordinates/frames and muscles

coord_list = {'flexion', 'mp_flexion', '2mcp_flexion', '3mcp_flexion'};
muscle_list = {'EDCI'}; 

coord_value = zeros(length(coord_list),1);%[0 0 0 1 1 0 1];
model.set_coordinate_value(coord_list,coord_value);
coord_q_value = model.get_coordinate_value(coord_list);

Jacobian_matrix = model.getJacobian_point('2distph', Vec3(0,0,0));
Jacobian_matrix2 = model.getJacobian_point('2distph', Vec3(0.1,0,0));
Jacobian_matrix_sub = model.getJacobian_point_sub('2distph', Vec3(0.1,0,0),coord_list);
%

%% get coordinate axis 
joint = model.JointSet.get('CMC1a');
coord = joint.getCoordinate();
coord.getName();
coord.getMotionType()

CJ = org.opensim.modeling.CustomJoint.safeDownCast(joint)
spatialTransform = CJ.getSpatialTransform();

a = spatialTransform.get_rotation1

%%
w_aor_all = model.get_coordinate_axis_all()
w_x_all = model.get_coordinate_pos_all()
%%
a = model.CoordinateSet.get(coord_list{2})
a = model.JointSet.get('CMC1a')
b = a.get_coordinates(0)
c = b
pfo = model.getJointSet().get('femur_femur_distal_r').get_frames(0).clone()


coord_list{1};
b = a.getJoint;
c = b.get_coordinates(0);

%% get muscle path
mus_name_list = {'EDCI'}; 
[PathPoints] = model.get_musclePathPoint(mus_name_list);
% return
%% Visualization
model.set_coordinate_value(model.CoordinateSet_list(:,1),0.5*zeros(model.CoordinateSet.getSize(),1));

% model.model_visualize
% model.plot_all_body;
model.plot_all_body_frame;
model.plot_world_frame;
% model.plot_mp_frame;
model.plot_all_Coordinate(0.02)
%% Jacobian


coord_value = [1 0 0 0];%[0 0 0 1 1 0 1];
model.set_coordinate_value(coord_list,coord_value);
Jacobian_all_trans2 = model.getJacobian_mp_all_trans(1);
state_value2 = model.get_systemStateValues;


%% Mass matrix
M_all = model.getMassMatrix_all;
M_sub = model.getMassMatrix_sub(coord_list);

%% Muscle parameters

a = model.MuscleSet.get(muscle_list{1});

OFL = model.get_OptimalFiberLength(muscle_list);
FL = model.get_FiberLength(muscle_list);
TL = model.get_TendonLength(muscle_list);
ML = model.get_muscleLength(muscle_list);
b = a.getFiberLengthAlongTendon(model.state);
% a.get_tendon_slack_length(model.state)
% b + TL


MIF = model.get_MaxIsometricForce(muscle_list);
TF = model.get_TendonForce(muscle_list);
FF = model.get_FiberForce(muscle_list);
FFAT = model.get_FiberForceAlongTendon(muscle_list);
PFF = model.get_PassiveFiberForce(muscle_list);
AFF = model.get_ActiveFiberForce(muscle_list);

b = a.getFiberForceAlongTendon(model.state);
AFLf = model.get_activeforcelengthfactor(muscle_list);

%% test muscle fiber length state
coord_list = {'flexion', '2mcp_flexion', '2pm_flexion','2md_flexion'};
m = model.MuscleSet.get(muscle_list{1});
q1 = model.CoordinateSet.get('2mcp_flexion');
model.set_coordinate_value({'2mcp_flexion'},0);
a = q1.getValue(model.state);
 
FL = model.get_FiberLength(muscle_list);
FF = model.get_FiberForce(muscle_list);
TL = model.get_TendonLength(muscle_list);
s = q1.getSpeedValue(model.state);
v = m.getFiberVelocity(model.state);
% change q
q1.setValue(model.state,0.6);
FL1 = model.get_FiberLength(muscle_list);
FF1 = model.get_FiberForce(muscle_list);
TL1 = model.get_TendonLength(muscle_list);
s1 = q1.getSpeedValue(model.state);
v1 = m.getFiberVelocity(model.state);
model.model.equilibrateMuscles(model.state);
FL2 = model.get_FiberLength(muscle_list);
FF2 = model.get_FiberForce(muscle_list);
TL2 = model.get_TendonLength(muscle_list);
s2 = q1.getSpeedValue(model.state);
v2 = m.getFiberVelocity(model.state);
% change qdot
q1.setSpeedValue(model.state,0.4);
model.model.equilibrateMuscles(model.state);
FL3 = model.get_FiberLength(muscle_list);
FF3 = model.get_FiberForce(muscle_list);
TL3 = model.get_TendonLength(muscle_list);
s3 = q1.getSpeedValue(model.state);
v3 = m.getFiberVelocity(model.state);

%% Moment arm matrix
% FCU muscle has moment arm at shoulder joint
MA_matrix = model.get_MomentArmMatrix(coord_list, muscle_list);


%% inverse kinematic
% iterative ik

if IK_on
    close all
    % q_init = model.get_coordinate_value(coord_list);
    q_des = 0.1*rand(7,1);
    model.set_coordinate_value(coord_list,q_des);
    x_p_des = model.get_mp_frame(1);
    q_init = 1*rand(7,1);
    model.set_coordinate_value(coord_list,q_init);

    model.plot_all_body;
    model.plot_world_frame;
    model.plot_mp_frame;
    model.plot_frame(x_p_des(1:3), euler2R_XYZ(x_p_des(4:6)),0.15);

    [q,x_res,phi_x,iter] = model.ik_numeric( coord_list, 1, x_p_des,200, [1e-4*ones(3,1);1e-2*ones(3,1)],0.15);
    % model.plot_mp_frame;
end

%% system state

state_name = model.get_systemStateNames;
state_value = model.get_systemStateValues;

%% scaling 
% 
% Scaling_file = '.\Scale_Setup_05_2.xml';
% model.set_scalefile(Scaling_file);
% 
% model.show_scale_info;
% model.scale_tool.run()

% %% generate scaling setting file 
% %TODO: parameters merge into the class funciton
% 
% model = osim_model('D:\repos\opensim_hand_project\model\WristModel_Marker_AllMuscles.osim');
% model.model.setName('WristModel_Marker_AllMuscles');
% 
% 
% model.generate_scaling_setup_file('statics_05.trc', '05', 4, 4.79333, '.\results')
% %% scale the model 
% % TODO .osim should be in the same folder 
% model.set_scalefile('.\scaled_setup_subject_05.xml')
% model.show_scale_info;
% model.scale_tool.run()






