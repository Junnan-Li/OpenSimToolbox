



clear all
close all
clc

import org.opensim.modeling.*

model_folder_path = 'D:\repos\opensim_hand_project\model\';
geometry_folder_path = 'D:\repos\opensim_hand_project\Geometry\';
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);

%% load model

model = osim_model('D:\repos\opensim_hand_project\model\WristModel_Marker_AllMuscles.osim');
set2list(model.muscleSet);
model.list_elements();

% set2list(model2.muscleSet)


return
%%
% model.muscleSet.get(2)
for i = 1:model.noutput
    model.muscleSet.remove(model.muscleSet.get(0));
    model.forceSet.remove(model.forceSet.get(0));
end
% model.update_set();
% set2list(model.muscleSet)
% set2list(model.model.getMuscles())
model.list_elements
return
%%

state = model.model.initSystem();
model.model.finalizeConnections();
model_save_path = 'D:\repos\opensim_hand_project\examples\model_save.osim';
model.save_model(model_save_path)

set2list(model.model.getMuscles());

%% scaling 
% 
% Scaling_file = '.\Scale_Setup_05_2.xml';
% model.set_scalefile(Scaling_file);
% 
% model.show_scale_info;
% model.scale_tool.run()

%% generate scaling setting file 
%TODO: parameters merge into the class funciton

model = osim_model('D:\repos\opensim_hand_project\model\WristModel_Marker_AllMuscles.osim');
model.model.setName('WristModel_Marker_AllMuscles');


model.generate_scaling_setup_file('statics_05.trc', '05', 4, 4.79333, '.\results')
%% scale the model 
% TODO .osim should be in the same folder 
model.set_scalefile('.\scaled_setup_subject_05.xml')
model.show_scale_info;

model.scale_tool.run()






