



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
%% add endefector/interest points as marker_point 

% body_name = {'distal_thumb'}; % name of the attached body
% pos_vec = {Vec3(0.002,-0.017,0)}; % relative position in corresponding body frame
% model.add_marker_points('Fingertip1', body_name,pos_vec);
% 
% body_name = {'2distph'}; % name of the attached body
% pos_vec = {Vec3(0.002,-0.017,0)}; % relative position in corresponding body frame
% model.add_marker_points('Fingertip2', body_name,pos_vec);
% 
% body_name = {'3distph'}; % name of the attached body
% pos_vec = {Vec3(0.002,-0.017,0)}; % relative position in corresponding body frame
% model.add_marker_points('Fingertip3', body_name,pos_vec);
% 
% body_name = {'4distph'}; % name of the attached body
% pos_vec = {Vec3(0.002,-0.017,0)}; % relative position in corresponding body frame
% model.add_marker_points('Fingertip4', body_name,pos_vec);
% 
% body_name = {'5distph'}; % name of the attached body
% pos_vec = {Vec3(0.002,-0.017,0)}; % relative position in corresponding body frame
% model.add_marker_points('Fingertip5', body_name,pos_vec);
model.set_visualize;
% return
%% set the list of the target coordinates/frames and muscles

% muscle_list = {'ECRL','ECRB','ECU','FCR','FCU','EDCI'};
% coord_list = {'2mcp_abduction','2mcp_flexion', '2pm_flexion', '2md_flexion'};
% coord_list = {'flexion', 'mp_flexion', '2mcp_flexion', '3mcp_flexion'};
coord_list = {'2mcp_flexion'};
muscle_list = {'EDCI'}; 
% state_value = model.get_systemStateValues;
coord_value = zeros(1,1);%[0 0 0 1 1 0 1];
model.set_coordinate_value(coord_list,coord_value);
coord_q_value = model.get_coordinate_value(coord_list);
% state_value = model.get_systemStateValues;

%% test muscle length

coord_list = {'2mcp_abduction','2mcp_flexion', '2pm_flexion', '2md_flexion'};
muscle_list = {'ECRL','ECRB','ECU','FCR','FCU','EDCI'};




%% test

body_list = {'humerus', 'ulna', 'radius','firstmc',...
    'secondmc','2proxph','2midph','2distph',...
    'thirdmc','3proxph','fourthmc','4proxph','fifthmc'}
model.plot_body(body_list)
mcp2 = model.BodySet.get('secondmc')

a = model.CoordinateSet.get('2md_flexion')
b = model.CoordinateSet.get('3md_flexion')
a.getBodyIndex
b.getBodyIndex
% a_J = a.getJoint


coord_list = {'flexion', 'mp_flexion', '2mcp_flexion', '3mcp_flexion'};
Jacobian_matrix = model.getJacobian_point('2distph', Vec3(0,0,0))
Jacobian_matrix2 = model.getJacobian_point('2distph', Vec3(0.1,0,0))

% coord_list = {'2pm_flexion'};
% model.set_coordinate_value(coord_list,0.2);

model.update_system
Jacobian_m = Matrix();
body_index = model.BodySet.get('3distph').getMobilizedBodyIndex();           
model.smss.calcFrameJacobian(model.state, body_index, Vec3(0), Jacobian_m);
Jacobian_matrix = osimMatrix2matrix(Jacobian_m);



return
%% Visualization
model.model_visualize
model.plot_all_body;
model.plot_world_frame;
model.plot_mp_frame;

%% Jacobian
% Jacobian_all1 = model.getJacobian_mp_all(1) % Jacobian of all coordinates

% Jacobian_all_trans1 = model.getJacobian_mp_all_trans(1);
% Jacobian_all_trans2 = model.getJacobian_mp_all_trans(2);
% Jacobian_all_trans3 = model.getJacobian_mp_all_trans(3);
% Jacobian_all_trans4 = model.getJacobian_mp_all_trans(4);
% Jacobian_all_trans5 = model.getJacobian_mp_all_trans(5);

coord_value = [1 0 0 0];%[0 0 0 1 1 0 1];
model.set_coordinate_value(coord_list,coord_value);
Jacobian_all_trans2 = model.getJacobian_mp_all_trans(1);
state_value2 = model.get_systemStateValues;

coord_list = {'flexion', 'mp_flexion', '2mcp_flexion', '3mcp_flexion'};
Jacobian_sub = model.getJacobian_point_sub(5,coord_list ); % Jacobian of selected coord.

%% Mass matrix
M_all = model.getMassMatrix_all;
M_sub = model.getMassMatrix_sub(coord_list);

%% Muscle parameters
mus_MIF_vec = model.get_MaxIsometricForce(muscle_list);
mus_PTF_vec = model.get_PassiveTendonForce(muscle_list);
mus_PA_vec = model.get_PennationAngle( muscle_list);
mus_ML_vec = model.get_muscleLength(muscle_list);

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






