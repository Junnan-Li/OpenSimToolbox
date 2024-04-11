








clear all
clc


import org.opensim.modeling.*

folder_path = pwd;
geometry_folder_path = strcat(folder_path,'\geometry_folder\Geometry_MoBL_ARMS\');
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);

model_a = Model(strcat(folder_path,'\model\MOBL_ARMS_fixed_41.osim'));
model_a.setUseVisualizer(false);
% a = model_a.updVisualizer();

state = model_a.initSystem();


model_a.equilibrateMuscles(state);

% ---------------------------------------------------------------------------
% Configure the visualizer
% ---------------------------------------------------------------------------

viz = model_a.updVisualizer().updSimbodyVisualizer();
viz.setBackgroundColor(Vec3(0)); % white
viz.setGroundHeight(-2)
viz.setShowSimTime(true);
model_a.getVisualizer().show(state);





