



clear all
close all
clc

import org.opensim.modeling.*

folder_path = pwd;
geometry_folder_path = strcat(folder_path,'\geometry_folder\Geometry_MoBL_ARMS\');
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);


IK_on = 1;
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
model.set_visualize;
%% set the list of the target coordinates/frames and muscles

muscle_list = {'ECRL','ECRB','ECU','FCR','FCU','EDCI'};
coord_list = {'elv_angle','shoulder_elv', 'shoulder_rot', 'elbow_flexion',...
    'pro_sup','deviation','flexion'};

% disable constrains
% constrains_diasble = {'wrist_hand_r1_con', 'wrist_hand_r3_con'};
% model.set_constrain_disable(constrains_diasble);

coord_value = rand(7,1);%[0 0 0 1 1 0 1];
model.set_coordinate_value(coord_list,coord_value);
coord_q_value = model.get_coordinate_value(coord_list);
%% Visualization
model.model_visualize
model.plot_all_body;
model.plot_world_frame;
model.plot_mp_frame;

%% Jacobian

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


%% inverse kinematic test
% iterative ik
q_init = model.get_coordinate_value(coord_list);
q_des = 1*rand(7,1)-0.1; %coord_q_value + 
model.set_coordinate_value(coord_list,q_des);
x_p_des = model.get_mp_frame(1);
q_init = 1*rand(7,1);

if IK_on
    close all
    model.set_coordinate_value(coord_list,q_init);
%     figure(1)
%     model.plot_all_body;
%     model.plot_world_frame;
%     model.plot_mp_frame;
%     model.plot_frame(x_p_des(1:3), euler2R_XYZ(x_p_des(4:6)),0.15);

    % Newton Raphson
    par_ik = model.IK_numeric_par();
    tic
    [q,info1] = model.IK_numeric( coord_list, 1, x_p_des,par_ik);
    t1 = toc;
    % LM method
    model.set_coordinate_value(coord_list,q_init);
%     figure(2)
%     model.plot_all_body;
%     model.plot_world_frame;
%     model.plot_mp_frame;
%     model.plot_frame(x_p_des(1:3), euler2R_XYZ(x_p_des(4:6)),0.15);
    ik_lm_par = model.IK_numeric_LM_par(length(q_init));
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

%% system state

state_name = model.get_systemStateNames;
state_value = model.get_systemStateValues;
state_list = model.get_systemState;
%% Forward kinematic test

if FK_on
close all

% q_init = [0.1, 0.1,0.1,0.1,0.1,0.2,0.1];
q_init = [0.4, 0.2,0.1,0.4,0.1,0.2,0.1];
step = 300;
% q1 = model.CoordinateSet.get('wrist_hand_r1')
con1 = model.ConstraintSet.get('wrist_hand_r1_con')
% con1.setIsEnforced(model.state,0)

delta_x = [0.0002,0,0,0,0,0]'; 
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
    J1 = model.getJacobian_mp_sub(1,coord_list ); 
    if mod(i,5) == 0
%         model.plot_mp_frame;
        model.model_visualize
    end
    delta_q = pinv(J1) * delta_x;
    x_p(:,i) = model.get_mp_frame(1);
    q_all(:,i+1) = q_i + delta_q;
    state_name = model.get_systemStateNames;
    state_value = model.get_systemStateValues;

end 
end

%%

coord_list = {'deviation'};
q = 1;
model.set_coordinate_value(coord_list,q);
coord_q_value = model.get_coordinate_value({'wrist_hand_r1'})

state_name = model.get_systemStateNames;
state_value = model.get_systemStateValues;

a = model.ConstraintSet.get("wrist_hand_r1_con")
% b = model.CoordinateSet.get("wrist_hand_r1")
func = a.getPropertyByIndex(2).getValueAsObject(0);
x = func.getPropertyByIndex(0);
y = func.getPropertyByIndex(1);


% b = a.getPrescribedFunction

import org.opensim.modeling.*

f1 = SimmSpline();
f1.addPoint(-0.174533,-0.261799);
f1.addPoint(0,-0);
f1.addPoint(0.436332,0.436332);
f1.calcValue(Vector(StdVectorDouble(q)))
f1.calcDerivativeVector(StdVectorDouble(q))

%%

for i = 1:length(model.ConstraintSet_list)
    con_i = model.ConstraintSet.get(i-1);
    % b = model.CoordinateSet.get("wrist_hand_r1")
    coor_name = con_i.getPropertyByIndex(3).toString;
    coord_parent = char(coor_name.substring(1,coor_name.length-1));
    coord_child = char(con_i.getPropertyByIndex(4));
    coor_child_index = find(ismember(model.CoordinateSet_list(:,1),coord_child));
    model.CoordinateSet_list(coor_child_index,2) = {coord_parent};

    func = con_i.getPropertyByIndex(2).getValueAsObject(0);
    x = func.getPropertyByIndex(0);
    y = func.getPropertyByIndex(1);
    model.CoordinateSet_list(coor_child_index,3) = x.toString;
    model.CoordinateSet_list(coor_child_index,4) = y.toString;
end

