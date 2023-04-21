function [locations, errors, angles,anglesPacked, momentarm] = solverIK(model, trcpath, markerList, weights, accuracy)
%% solverIK is a Matlab interface to the OpenSim IKSolver(). 
% Inputs are an OpenSim Model, path to a trc file, list of markers to be
% tracked, and an array of weights corresponding with each marker. 
%
% Outputs are the locations of the model markers, the errors between the
% model and experimental markers, a coordinate value array (without labels),
% and a struct of coordinate values (allocated labels).
% 

% use the IK solver and report back the individual marker errors. 
import org.opensim.modeling.*
% Initialize OpenSim model
s = model.initSystem();
nM = length(markerList);
%% get lists of the marker and coordinate names
% markerNames = [];
% for i = 0 : model.getMarkerSet().getSize()-1;
%    markerNames = [markerNames {char(model.getMarkerSet().get(i).getName() )}];    
% end
% 
% trcNames = [];
% for i = 0 : t.getNumColumns() - 1
%     trcNames = [trcNames {char(t.getColumnLabels().get(i))}];
% end
% Generate a final list of Markers to track
% markerList = intersect(markerNames,trcNames);

coordName = [];
for i = 0 : model.getCoordinateSet().getSize()-1;
   coordName = [coordName {char(model.getCoordinateSet().get(i).getName() )}];    
end

%% Populate markers reference object with desired marker weights
markerWeights = SetMarkerWeights();
for i = 1 : length(markerList) 
    markerWeights.cloneAndAppend( MarkerWeight(markerList{i}, weights(i) ) );
end

% Set marker weights
markerref = MarkersReference(trcpath,markerWeights);
% markerref = MarkersReference();
markerref.setMarkerWeightSet(markerWeights)
% Create an arry of blank coordinate reference objects
coordref = SimTKArrayCoordinateReference();

% Populate coordinate references object with desired coordinate values
% and weights
for i = 1 : length(coordName)
    coordRef = CoordinateReference(coordName{i} , Constant() );
    coordRef.setWeight(0);
    coordRef.markAdopted();
    coordref.push_back(coordRef);
end

%% Define kinematics reporter for output
% Load marker data
% markerref.loadMarkersFile(trcpath) % , Units('Millimeters'));
% markerref.loadMarkersFile(trcpath)
% markerref.set_marker_file(trcpath)

% Get start and end times
timeRange = markerref.getValidTimeRange();
startTime = timeRange.get(0);
endTime = timeRange.get(1);

% Define constraint weight
constweight = Inf;

%% Instantiate the ikSolver and set some values. 
ikSolver = InverseKinematicsSolver(model,markerref, coordref,constweight);
% Set ikSolver accuracy

% accuracy = 1e-5;
ikSolver.setAccuracy(accuracy);
% Assemble the model
s.setTime(startTime);
ikSolver.assemble(s);

%% Loop through markers and define weights
% for i = 0 : model.getMarkerSet().getSize() - 1
%     ikSolver.updateMarkerWeight(i,1);
% end

% Loop through IK coordinate tasks and define values and weights
for i = 1 : model.getCoordinateSet().getSize()
    ikSolver.updateCoordinateReference(coordName{i},0);
end

%% Compute dt and nFrames
dt = 1.0/markerref.getSamplingFrequency();
nFrames = round(( (endTime-startTime)/dt )+1);

%% Perform IK analysis
disp('Starting IK analysis')
errors = [];
angles = [];
locations = [];
momentarm = [];
for i = 0 : nFrames - 1
    % set the time
    s.setTime(startTime+i*dt);
    % run the iksolver for the time
    ikSolver.track(s);
    % Get the marker errors
    for u = 0 : nM - 1
           errors(i+1,u+1) =  ikSolver.computeCurrentMarkerError(u);
    end
    % Get the Model Marker Locations
    for u = 0 : nM - 1
            location =  ikSolver.computeCurrentMarkerLocation(u);
            locations(i+1,(u+1)*3-2:(u+1)*3) = [location.get(0) location.get(1) location.get(2)]; 
    end
    % Get the Coordinates Values
    model.realizeVelocity(s);
    % Get the coordinate Set
    cs = model.getCoordinateSet();
    % 
    FDS4 = model.getMuscles().get('FDS4');
    momentarmi = FDS4.computeMomentArm(s,cs.get('MCP4_flex'));
    momentarm(i+1) = momentarmi;
    
    nvalues = cs.getSize();
    for u = 0 : nvalues - 1
        angles(i+1,u+1) = cs().get(u).getValue(s);
    end
end

%% Pack the angles 
anglesPacked = struct();
for u = 0 : nvalues - 1
    name = char(cs.get(u).getName());
    if ~isempty(strfind(name,'tx')) || ~isempty(strfind(name,'ty')) || ~isempty(strfind(name,'tz'))
        eval(['anglesPacked.' char(cs.get(u).getName()) '= angles(:,u+1);'])
    else
        % Angles are in rad. COnvert to deg
        if strcmp( cs.get(u).getName(),'auxprfem-auxprrud_coord' )
            eval(['anglesPacked.uxprfem_auxprrud_coord = rad2deg(angles(:,u+1));'])
        else
            eval(['anglesPacked.' char(cs.get(u).getName()) '= rad2deg(angles(:,u+1));'])
        end
    end
end

anglesPacked.time = [startTime:dt:endTime]';


end