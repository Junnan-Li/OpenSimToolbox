% Path Initial 
% 
%
%

% Path of the Repo should be change 
filepath = fileparts(mfilename('fullpath'));

Path_functions = strcat(filepath, '\functions');
Path_classes = strcat(filepath, '\classes');
Path_Geometry = strcat(filepath, '\geometry_folder');
% Path_Code_tool = strcat(filepath, '\Code\matlabtool');
Path_Model = strcat(filepath, '\model');
Path_Metrics = strcat(filepath, '\metrics');

Path_example= strcat(filepath, '\examples');

addpath(genpath(Path_functions));
addpath(Path_classes);
addpath(genpath(Path_Metrics));
addpath(Path_Model);
addpath(Path_Geometry);

addpath(Path_example);