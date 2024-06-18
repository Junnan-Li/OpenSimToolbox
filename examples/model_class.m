



clear all
close all
clc

import org.opensim.modeling.*

folder_path = pwd;
geometry_folder_path = strcat(folder_path,'\geometry_folder\Geometry_MoBL_ARMS\');
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);


IK_on = 1;
FK_on = 1;

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
pos_vec = {Vec3(0,-0,0)}; % relative position in corresponding body frame
model.add_marker_points('Hand_endeffector', body_name,pos_vec);
model.set_visualize;
%% set the list of the target coordinates/frames and muscles

muscle_list = {'ECRL','ECRB','ECU','FCR','FCU','EDCI'};
% coord_list = {'elv_angle','shoulder_elv', 'shoulder_rot', 'elbow_flexion',...
%     'pro_sup','deviation','flexion'};
coord_list = model.Coord_minimal(:,1); % get the minimal set of coordinates

coord_value = rand(7,1);%[0 0 0 1 1 0 1];
model.set_coordinate_value(coord_list,coord_value);
coord_q_value = model.get_coordinate_value(coord_list);
%% Visualization
model.model_visualize
model.plot_all_body;
model.plot_world_frame;
model.plot_mp_frame;

%% Jacobian
% normally we use the minimal set of coordinates
% whose Jacobian is
J_syn = model.getJacobian_mp_minimal(1 );


% get all other kind of Jacobian (trancated, without considering constraints)
Jacobian_point = model.getJacobian_point(body_name, Vec3(0,-0.1,0) );
Jacobian_point_sub = model.getJacobian_point_sub(body_name, Vec3(0,-0.1,0),coord_list );
Jacobian_all = model.getJacobian_mp_all(1); % Jacobian of all coordinates
Jacobian_sub = model.getJacobian_mp_sub(1,coord_list ); % Jacobian of selected coord.
%% Mass matrix

M_all = model.getMassMatrix_all;
M_sub = model.getMassMatrix_sub(coord_list);

%% Muscle parameters
mus_MIF_vec = model.get_MaxIsometricForce(muscle_list);
mus_PTF_vec = model.get_TendonForce(muscle_list);
mus_PA_vec = model.get_PennationAngle( muscle_list);
mus_ML_vec = model.get_muscleLength(muscle_list);
mus_FF_vec = model.get_FiberForce(muscle_list); % fiber force
mus_FFAT_vec = model.get_FiberForceAlongTendon(muscle_list);% fiber force along tendon
%% Moment arm matrix
% FCU muscle has moment arm at shoulder joint
MA_matrix = model.get_MomentArmMatrix(coord_list, muscle_list);

%% system state

state_name = model.get_systemStateNames;
state_value = model.get_systemStateValues;
state_list = model.get_systemState;

%% inverse kinematic test
% iterative ik

q_des = diff(model.Coord_minimal_range,1,2) .* rand(7,1) + model.Coord_minimal_range(:,1);
model.set_coordinate_value(coord_list,q_des);
q_des = model.get_coordinate_value(coord_list);
x_p_des = model.get_mp_frame(1);
q_init = diff(model.Coord_minimal_range,1,2) .* rand(7,1) +model.Coord_minimal_range(:,1);

if IK_on
    close all
    model.set_coordinate_value(coord_list,q_init);
    figure(1)
    model.plot_all_body;
    model.plot_world_frame;
    model.plot_mp_frame;
    model.plot_frame(x_p_des(1:3), euler2R_XYZ(x_p_des(4:6)),0.15);

    % Newton Raphson
    par_ik = model.IK_numeric_par();
    par_ik.visual = 1;
    par_ik.alpha = 0.3;
    tic
    [q,info1] = model.IK_numeric( coord_list, 1, x_p_des,par_ik);
    t1 = toc;
    % LM method
    model.set_coordinate_value(coord_list,q_init);
    figure(2)
    model.plot_all_body;
    model.plot_world_frame;
    model.plot_mp_frame;
    model.plot_frame(x_p_des(1:3), euler2R_XYZ(x_p_des(4:6)),0.15);
    ik_lm_par = model.IK_numeric_LM_par(length(q_init));
    ik_lm_par.visual = 1;
    tic
    ik_lm_par.W_d = 1e-7*diag(ones(length(q_init),1));
    [q2,info2] = model.IK_numeric_LM(coord_list, 1, x_p_des,ik_lm_par);
    t2 = toc;

    % adaptive LM not finished
%     model.set_coordinate_value(coord_list,q_init);
%     tic
%     [q3,info3] = model.IK_numeric_LM_adaptive(coord_list, 1, x_p_des, ...
%                 diag([1,1,1,1,1,1]),200, [1e-4*ones(3,1);1e-2*ones(3,1)]);

    fprintf('IK test: \n')
    fprintf('Method 1 (NR): status: %d; time: %f; iter: %d \n', info1.status,t1,info1.iter)
    fprintf('Method 2 (LM): status: %d; time: %f; iter: %d \n', info2.status,t2,info2.iter)
end


%% Forward kinematic test

if FK_on
    close all

    % q_init = [0.1, 0.1,0.1,0.1,0.1,0.2,0.1];
    q_init = diff(model.Coord_minimal_range,1,2) .* rand(7,1) + model.Coord_minimal_range(:,1);
    step = 200;

    delta_x = [0.0002,-0.0,0,0,0,0]';
    model.set_coordinate_value(coord_list,q_des);
    x_init = model.get_mp_frame(1);
    q_all = zeros(length(coord_list),step);
    q_all(:,1) = q_init;
    x_p = zeros(6,step);
    figure(1)
    model.plot_world_frame;
    for i = 1:step
        q_i = q_all(:,i);
        model.set_coordinate_value(coord_list,q_i);
        J1 = model.getJacobian_mp_minimal(1 );
        if mod(i,5) == 0
            model.plot_mp_frame;
%             model.model_visualize
        end
        delta_q = pinv(J1) * delta_x;
        x_p(:,i) = model.get_mp_frame(1);
        q_all(:,i+1) = q_i + delta_q;
        state_name = model.get_systemStateNames;
        state_value = model.get_systemStateValues;

    end
end
