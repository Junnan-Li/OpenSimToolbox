



clear all
close all
clc

import org.opensim.modeling.*

folder_path = pwd;
geometry_folder_path = strcat(folder_path,'\geometry_folder\Geometry_MoBL_ARMS\');
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);


IK_on = 0;
FK_on = 0;

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
% model.set_visualize;
%% set the list of the target coordinates/frames and muscles

muscle_list = {'ECRL','ECRB','ECU','FCR','FCU','EDCI'};
% coord_list = {'elv_angle','shoulder_elv', 'shoulder_rot', 'elbow_flexion',...
%     'pro_sup','deviation','flexion'};
coord_list = model.Coord_minimal(:,1); % get the minimal set of coordinates

coord_value = rand(7,1);%[0 0 0 1 1 0 1];
model.set_coordinate_value(coord_list,coord_value);
coord_q_value = model.get_coordinate_value(coord_list);
%% Visualization
% model.model_visualize
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

% M_all = model.getMassMatrix_all;
% M_sub = model.getMassMatrix_sub(coord_list);

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
% set a point close to the initial position
q_init = q_des-0.2*rand(7,1); 

model.set_coordinate_value(coord_list,q_init);
x_p_init = model.get_mp_frame(1);
model.Constraint_on = 0;
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
    par_ik.visual = 0;
    par_ik.alpha = 0.2;
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
    ik_lm_par.visual = 0;
    tic
    ik_lm_par.W_d = 1e-4*diag(ones(length(q_init),1));
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

%% test metric
close all
model.Constraint_on = 1;
% joint limits
[metric_JL,metric_JL_joints] = model.metric_joint_limits;
q = model.get_coordinate_value_minimal;
% model.set_coordinate_value_minimal([2.26,0,0,0,0,0,1.22]')
% metric_JL2 = model.metric_joint_limits
% q2 = model.get_coordinate_value_minimal;



% Dynamic Manipulability
% xdd = J*inv(M)*A * (F_MIF * m * alpha + F_p )
% for acceleration
q = rand(7,1);
q(6) = 0.4 * rand(1);
model.set_coordinate_value_minimal(q);

% Jacobian 
J = model.getJacobian_mp_minimal(1);
% Mass matrix
M = model.get_MassMatrix_minimal;
% Moment arm matrix 
MA = model.get_MomentArmMatrix_minimal_AllMus;
% Maximal Isomatrix force as diagonal matrix
F_MIF = model.get_MaxIsometricForce_all_diag;
% Force Length Multiplier as diagonal matrix
mus_FLM = model.get_FiberForceLengthMultiplier;
% get Passive force vector
F_P = model.get_PassiveFiberForce();

% Calculate maximal Cartesian acceleration vector 
opt = init_metric_opt();
% fmincon method: 
n_iter = 10;
acc_fmc = zeros(6,n_iter);
for i = 1:n_iter
    [x_fmc,acc_fmci] = cal_max_acc(model,opt);
    acc_fmc(:,i) = acc_fmci(:,1);
end
% opt.method = 'lsqlin';
% [x_fmc_sub,acc_fmc_sub] = cal_max_acc_subdirection(model,opt);
% norm(acc_fmc);
% alpha = model.get_PennationAngle;
% mus_PLM = model.get_PassiveForceLengthMultiplier;
% mus_FVM = model.get_FiberForceVelocityMultiplier;

% Calculate the maximal acceleration in axes direction
opt.method = 'global';
[x_acc_global,acc_fmc_global] = cal_max_acc_subdirection(model,opt);


% Calculate the maximal acceleration in all directions
% unfinished
% opt.method = 'global';
% opt.only_translational = 1;
% [x_fmc_global,acc_fmc_global] = cal_max_acc_radius(model,opt);


% Force index
% calculate maximal force resistance along each direction
opt.method = 'global';
[x_F_global,F_global] = cal_max_force_subdirection(model,opt);



return
%% Test muscle formulation

model.set_Activation_all(x_fmc(:,1));
F_MIF = model.get_MaxIsometricForce_all_diag;
alpha = model.get_PennationAngle;
mus_FLM = model.get_FiberForceLengthMultiplier;
mus_PLM = model.get_PassiveForceLengthMultiplier;
mus_FVM = model.get_FiberForceVelocityMultiplier;
f_t = F_MIF * (mus_FLM .* x_fmc(:,1) + mus_PLM) .*cos(alpha);
f_ta = F_MIF * mus_FLM .* x_fmc(:,1);% .*mus_FVM;
f_tp = F_MIF * mus_PLM;
f_end = pinv(J')*MA*f_t;



mus_act = model.get_Activation_all;

% alpha2 = model.get_PennationAngle;
f_ten = model.get_TendonForce;
f_ten_a = model.get_ActiveFiberForce;
f_ten_p = model.get_PassiveFiberForce;
f_ten2 = (f_ten_a + f_ten_p).*cos(alpha);
f_t_error = f_t - f_ten;
f_ta_error = f_ta - f_ten_a;
f_tp_error = f_tp - f_ten_p;

% Test formulation of muscle force calculation 
n_loop = 10;
F_MIF = zeros(50,n_loop);
alpha = zeros(50,n_loop);
mus_FLM = zeros(50,n_loop);
mus_PLM = zeros(50,n_loop);
mus_FVM = zeros(50,n_loop);
a = zeros(50,n_loop);
for i = 1:10
    a(:,i) = rand(50,1);
    model.set_Activation_all(a(:,i));
    F_MIF(:,i) = model.get_MaxIsometricForce;
    alpha(:,i) = model.get_PennationAngle;
    mus_FLM(:,i) = model.get_FiberForceLengthMultiplier;
    mus_PLM(:,i) = model.get_PassiveForceLengthMultiplier;
    mus_FVM(:,i) = model.get_FiberForceVelocityMultiplier;
end



%% 

muscle_list = {model.MuscleSet_list(10),model.MuscleSet_list(16)}';
q = rand(7,1);
q(6) = 0.4 * rand(1);
model.set_coordinate_value_minimal(q);

% Jacobian 
J = model.getJacobian_mp_minimal(1);
% Mass matrix
M = model.get_MassMatrix_minimal;
% Moment arm matrix 
MA = model.get_MomentArmMatrix_minimal_AllMus;
% Maximal Isomatrix force as diagonal matrix
F_MIF = model.get_MaxIsometricForce_all_diag;
% Force Length Multiplier as diagonal matrix
mus_FLM = model.get_FiberForceLengthMultiplier;
% get pennetion angle
alpha = model.get_PennationAngle;

% get Passive force vector
F_P = F_MIF *mus_PLM;

% set minimal actuation to 0 (not consistent to opensim)
F_min = F_P .*cos(alpha);
F_max = F_MIF * (mus_FLM + mus_PLM) .*cos(alpha);


muscle_index = [10:38];

force_limits = [F_min, F_max];


% [P_tau] = polytope_torque(MA(:,muscle_index), force_limits(muscle_index,:));
% P_tau.minHRep;
% P_tau.minVRep;
% P_tau.volume
% 
% P_f = pinv(J') * P_tau

