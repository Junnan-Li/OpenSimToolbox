clear all
close all
clc

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
mus_name_list = {'EPB','FDPI','EDCI'}; 
[PathPoints] = model.get_musclePathPoint(mus_name_list(1));

%% Visualization
% model.model_visualize
model.plot_all_body;
model.plot_all_body_frame;
model.plot_world_frame;
model.plot_muscle_path(mus_name_list);