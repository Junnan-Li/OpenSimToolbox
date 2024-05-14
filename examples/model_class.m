



clear all
close all
clc

import org.opensim.modeling.*

folder_path = pwd;
geometry_folder_path = strcat(folder_path,'\geometry_folder\Geometry_MoBL_ARMS\');
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);

IK_on = 0;

%% load model

model = osim_model(strcat(folder_path,'\model\MOBL_ARMS_fixed_41.osim')); % MOBL_ARMS_fixed_41

% check body list, joint list, ...
model.BodySet_list;
model.JointSet_list;
model.CoordinateSet_list;
model.MuscleSet_list;

% delete markers
model.delete_all_markers;
%% add endefector/interest points as marker_point 
body_name = {'hand'}; % name of the attached body
pos_vec = {Vec3(0,-0.1,0)}; % relative position in corresponding body frame
model.add_marker_points('Hand_endeffector', body_name,pos_vec);
model.set_visualize;
%% set the list of the target coordinates/frames and muscles

muscle_list = {'ECRL','ECRB','ECU','FCR','FCU','EDCI'};
coord_list = {'elv_angle','shoulder_elv', 'shoulder_rot', 'elbow_flexion',...
    'pro_sup','deviation','flexion'};

coord_value = rand(7,1);%[0 0 0 1 1 0 1];
model.set_coordinate_value(coord_list,coord_value);
coord_q_value = model.get_coordinate_value(coord_list);
%% Visualization
model.model_visualize
model.plot_all_body;
model.plot_world_frame;
model.plot_mp_frame;

%% Jacobian
Jacobian_all = model.getJacobian_mp_all(1); % Jacobian of all coordinates
Jacobian_sub = model.getJacobian_point_sub(1,coord_list ); % Jacobian of selected coord.

%% Mass matrix

M_all = model.getMassMatrix_all;
M_sub = model.getMassMatrix_sub(coord_list);

%% Muscle parameters
mus_MIF_vec = model.get_MaxIsometricForce(muscle_list);
mus_PTF_vec = model.get_TendonForce(muscle_list);
mus_PA_vec = model.get_PennationAngle( muscle_list);
mus_ML_vec = model.get_muscleLength(muscle_list);
mus_FF_vec = model.get_fiberforce(muscle_list); % fiber force
mus_FFAT_vec = model.get_fiberforcealongtendon(muscle_list);% fiber force along tendon
%% Moment arm matrix
% FCU muscle has moment arm at shoulder joint
MA_matrix = model.get_MomentArmMatrix(coord_list, muscle_list);


%% inverse kinematic
% iterative ik

if IK_on
    close all
    % q_init = model.get_coordinate_value(coord_list);
    q_des = coord_q_value + 0.2*rand(7,1)-0.1;
    model.set_coordinate_value(coord_list,q_des);
    x_p_des = model.get_mp_frame(1);
    q_init = 1*rand(7,1);
    model.set_coordinate_value(coord_list,q_init);

    model.plot_all_body;
    model.plot_world_frame;
    model.plot_mp_frame;
    model.plot_frame(x_p_des(1:3), euler2R_XYZ(x_p_des(4:6)),0.15);

    [q,x_res,phi_x,iter] = model.ik_numeric( coord_list, 1, x_p_des,200, [1e-3*ones(3,1);5e-2*ones(3,1)],0.2);
    % model.plot_mp_frame;
end

%% system state

state_name = model.get_systemStateNames;
state_value = model.get_systemStateValues;

%% scaling 
% 

coord_list = {'deviation','flexion','wrist_hand_r1', 'wrist_hand_r3'};

q = [0.42,0,0,0]';
model.set_coordinate_value(coord_list,q);
coord_q_value = model.get_coordinate_value(coord_list)

a = model.CoordinateSet.get('deviation')
b = model.ConstraintSet.get('wrist_hand_r1_con')

