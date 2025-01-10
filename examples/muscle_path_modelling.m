clear all
close all
clc

%% load Majtaba hand model
run create_Hand_Model_wt_Muscles.m
close all

%%
import org.opensim.modeling.*

folder_path = pwd;
geometry_folder_path = strcat(folder_path,'\geometry_folder\Geometry_MoBL_ARMS\');
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);

%% load model

model = osim_model(strcat(folder_path,'\model\Hand_Wrist_Model_for_development.osim'));

% check body list, joint list, ...
model.BodySet_list;
model.JointSet_list;
model.CoordinateSet_list;
model.MuscleSet_list;

% delete markers
model.delete_all_markers;

%% get muscle path
mus_name_list = {'FDPI'}; 
[PathPoints] = model.get_musclePathPoint(mus_name_list(1));

%% Visualization
% model.model_visualize
figure(1)
model.plot_all_body;
model.plot_all_body_frame;
model.plot_world_frame;
model.plot_muscle_path(mus_name_list);

%
q0 = zeros(Mojtaba_hand.nj,1);
Mojtaba_hand.update_hand(q0);
plot_hand_par.markersize = 5;
Mojtaba_hand.plot_hand(plot_hand_par)
grid on 
axis equal

%% reorientation
% 
% Coordinate_list_ASMR = { ...
%     '2proxph'; ...
%     '2midph';...
%     '2distph';...
% };
 w_R = osimMatrix2matrix(model.model.getGround.getTransformInGround(model.state).R);


% index finger
t = Mojtaba_hand.list_fingers(2).get_w_T_base_inhand;
w_T_mojtaba_index = Mojtaba_hand.list_fingers(2).get_T_all_links_inhand;
w_R_MCP = w_T_mojtaba_index(1:3,1:3,2);
w_R_PIP = w_T_mojtaba_index(1:3,1:3,4);
w_R_DIP = w_T_mojtaba_index(1:3,1:3,5);
% plot3(t(1,4),t(2,4),t(3,4),'c.','MarkerSize',35)



w_T_2proxph = model.get_body_T('2proxph');
w_T_2midph = model.get_body_T('2midph');
w_T_2distph = model.get_body_T('2distph');
% plot3(T_MCP2(1,4),T_MCP2(2,4),T_MCP2(3,4),'c.','MarkerSize',35)
% w_x = T_PIP2(1:3,4) - T_MCP2(1:3,4);

w_x_index_Majtaba = w_R_MCP * [1;0;0];
w_x_index_AMSR = w_T_2midph(1:3,4)-w_T_2proxph(1:3,4);

gamma = acos(w_x_index_Majtaba'*w_x_index_AMSR/ ...
    (norm(w_x_index_Majtaba)*(norm(w_x_index_AMSR))));
w_R_wopensim = eul2rotm([0,0,-gamma],'XYZ');

Mojtaba_R_2proxph = w_R_MCP'*w_R_wopensim*w_T_2proxph(1:3,1:3);
Mojtaba_R_2midph = w_R_PIP'*w_R_wopensim*w_T_2midph(1:3,1:3);
Mojtaba_R_2distph = w_R_DIP'*w_R_wopensim*w_T_2distph(1:3,1:3);

for i = 7:8
    w_p_new = Mojtaba_R_2proxph * PathPoints{2,i}';
%     plot3(w_p_new(1),w_p_new(2),w_p_new(3),'b.','MarkerSize',10);
    Mojtaba_hand.list_fingers(2).list_links(1).add_viapoint_link('L1_VP_1',w_p_new);
end
w_p_new = Mojtaba_R_2midph *  PathPoints{2,9}';
Mojtaba_hand.list_fingers(2).list_links(3).add_viapoint_link('L1_VP_1',w_p_new);
w_p_new = Mojtaba_R_2distph * PathPoints{2,10}';
Mojtaba_hand.list_fingers(2).list_links(4).add_viapoint_link('L1_VP_1',w_p_new);

Mojtaba_hand.list_fingers(2).update_list_viapoints
Mojtaba_hand.update_list_viapoints;
% Mojtaba_hand.update_hand(q0);
Mojtaba_hand.plot_hand_viapoints(plot_hand_par)

% Mojtaba_hand.delete_all_viapoint










