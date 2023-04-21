% create a scale_setup_file.xml: 
%         ScaleTool: mass, age, height 
%         GenericModelMarker: Osimmodel file name, markerSet file name
%         ModelScaler: MeasurementSet
%         MarkerPlacer: 
        



clear all
close all
clc

import org.opensim.modeling.*


StaticName = 'statics_05.trc';
OutputModelName = '.\results\scaled_subject05.osim';
OutputMarkerModelName = 'scaled_subject05_marker.osim';
ScaleFileName = 'Scale_Setup_05_2.xml';
ModelName = 'scaled_subject_05';
outputmotion_name = 'subject_05_scaling_motion.mot';
output_scaleset_name = 'subject_05_scaleSet_applied.xml';
maximal_time = 4.79333;

% Geometry_path='..\..\Geometry';
% ModelVisualizer.addDirToGeometrySearchPaths(Geometry_path);
% modelpath = '..\..\model';
% wrist_model = strcat(modelpath,'\WristModel_with_WristMuscles.osim');
% OsimModel = Model(wrist_model);

Scale_file = ScaleTool('.\Setup_Scale_example.xml');

Scale_file.getDocumentFileName(); % 'subject01_Setup_Scale.xml'

Scale_file.getGenericModelMaker().setModelFileName('WristModel_Marker_AllMuscles.osim');
Scale_file.getGenericModelMaker().setMarkerSetFileName('markerfile.xml');
Scale_file.setName(ModelName);
Scale_file.setSubjectMass(4);
%% Model_Scaler settings
Model_scaler = Scale_file.getModelScaler();
Model_scaler.setMarkerFileName(StaticName);
% Model_scaler.setTimeRange(Vec3(0, 0.973333333));
Model_scaler.getTimeRange().set(0,0);
Model_scaler.getTimeRange().set(1,maximal_time); % maximal time
Model_scaler.getMarkerFileName() % 

Model_scaler.setOutputModelFileName(OutputModelName);
Model_scaler.setOutputScaleFileName('');

Measurement_Set = Model_scaler.getMeasurementSet();
% Model_scaler.setName('scaled_subject_19');


Measurement_Set.setName('changed MeasurementSetName');
% remove old MeasurementSet
MeasurementList = [];
for i = 0 : Measurement_Set.getSize() - 1
    MeasurementList = [MeasurementList; {char(Measurement_Set.get(i))}];
end

for i = 2 : length(MeasurementList)
    Measurement_Set.remove(Measurement_Set.get(MeasurementList{i}));
end



% get MarkerPairSet and clean old files
MS_ori = Measurement_Set.get(0);
MPS_ori = MS_ori.getMarkerPairSet(); %
MBS_ori = MS_ori.getBodyScaleSet();

MS_ori.setName('Proximalphalanx1');
MS_ori.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files

MP=MarkerPair(); % MarkerPair
MP.setMarkerName(0,'MP1');
MP.setMarkerName(1,'IP');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
% BS = MBS_ori.get(0);
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Proximalphalanx1');


%% Mesurement Proximalphalanx2 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();


MS.setName('Proximalphalanx2');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(0);
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'MP2');
MP.setMarkerName(1,'PP2');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(1);
    end
end
MBS_ori.get(0).setName('Proximalphalanx2');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement Proximalphalanx3 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Proximalphalanx3');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'MP3');
MP.setMarkerName(1,'PP3');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Proximalphalanx3');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement Proximalphalanx4 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Proximalphalanx4');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'MP4');
MP.setMarkerName(1,'PP4');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Proximalphalanx4');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement Proximalphalanx5 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Proximalphalanx5');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'MP5');
MP.setMarkerName(1,'PP5');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Proximalphalanx5');

Measurement_Set.cloneAndAppend(MS);


%% Mesurement medialphalanx2 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Medialphalanx2');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'PP2');
MP.setMarkerName(1,'DP2');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Medialphalanx2');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement medialphalanx3 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Medialphalanx3');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'PP3');
MP.setMarkerName(1,'DP3');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Medialphalanx3');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement medialphalanx4 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Medialphalanx4');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'PP4');
MP.setMarkerName(1,'DP4');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Medialphalanx4');

Measurement_Set.cloneAndAppend(MS);


%% Mesurement medialphalanx3 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Medialphalanx5');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'PP5');
MP.setMarkerName(1,'DP5');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Medialphalanx5');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement distalphalanx1 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Distalphalanx1');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'IP');
MP.setMarkerName(1,'FT1');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Distalphalanx1');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement distalphalanx2 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Distalphalanx2');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'DP2');
MP.setMarkerName(1,'FT2');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Distalphalanx2');

Measurement_Set.cloneAndAppend(MS);


%% Mesurement distalphalanx3 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Distalphalanx3');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'DP3');
MP.setMarkerName(1,'FT3');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Distalphalanx3');

Measurement_Set.cloneAndAppend(MS);


%% Mesurement distalphalanx4 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Distalphalanx4');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'DP4');
MP.setMarkerName(1,'FT4');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Distalphalanx4');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement distalphalanx5 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('Distalphalanx5');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'DP5');
MP.setMarkerName(1,'FT5');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('Distalphalanx5');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement metacarpal2 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('metacarpal2_length');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'CMC2');
MP.setMarkerName(1,'PP2');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('metacarpal2');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement metacarpal5 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('metacarpal5_length');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'CMC5');
MP.setMarkerName(1,'PP5');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('metacarpal5');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement metacarpal4 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('metacarpal4_length');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'CMC5');
MP.setMarkerName(1,'PP5');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('metacarpal4');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement metacarpal3 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('metacarpal3_length');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'CMC2');
MP.setMarkerName(1,'PP2');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('metacarpal3');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement metacarpal1 setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('metacarpal1');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'CMC1');
MP.setMarkerName(1,'MP1');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('metacarpal1');

Measurement_Set.cloneAndAppend(MS);


%% Mesurement ulna setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('ulna');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'EM');
MP.setMarkerName(1,'US');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('ulna');

Measurement_Set.cloneAndAppend(MS);


%% Mesurement radius setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('radius');
MS.setApply(true);

for i = 0 : MPS_ori.getSize()-1
    MPS_ori.remove(MPS_ori.get(i));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'EL');
MP.setMarkerName(1,'RS');
MPS_ori.cloneAndAppend(MP);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()-1
        MBS_ori.remove(MBS_ori.get(i));
    end
end
MBS_ori.get(0).setName('radius');

Measurement_Set.cloneAndAppend(MS);

%% Mesurement metacarpal setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('metacarpal_width');
MS.setApply(true);

for i = 1 : MPS_ori.getSize()
    MPS_ori.remove(MPS_ori.get(0));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'CMC2');
MP.setMarkerName(1,'CMC5');
MPS_ori.cloneAndAppend(MP);
MP_2 = MarkerPair(); % MarkerPair
MP_2.setMarkerName(0,'MP2');
MP_2.setMarkerName(1,'MP5');
MPS_ori.cloneAndAppend(MP_2);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()
        MBS_ori.remove(MBS_ori.get(0));
    end
end
% MBS_ori.get(0).setName('metacarpal2');
% BS_new_1 = MBS_ori.get(0).clone();
% BS_new_1.setName('metacarpal3');
% MBS_ori.cloneAndAppend(BS_new_1);
% BS_new_2 = MBS_ori.get(0).clone();
% BS_new_2.setName('metacarpal4');
% MBS_ori.cloneAndAppend(BS_new_2);
% BS_new_3 = MBS_ori.get(0).clone();
% BS_new_3.setName('metacarpal5');
% MBS_ori.cloneAndAppend(BS_new_3);

% Measurement_Set.cloneAndAppend(MS);

%% Mesurement ulna width setting

MS = Measurement_Set.get(0).clone();
MPS_ori = MS.getMarkerPairSet(); %
MBS_ori = MS.getBodyScaleSet();

MS.setName('ulna_width');
MS.setApply(true);

for i = 1 : MPS_ori.getSize()
    MPS_ori.remove(MPS_ori.get(0));
end

% create new files
MP = MarkerPair(); % MarkerPair
MP.setMarkerName(0,'US');
MP.setMarkerName(1,'RS');
MPS_ori.cloneAndAppend(MP);
MP_2 = MarkerPair(); % MarkerPair
MP_2.setMarkerName(0,'EM');
MP_2.setMarkerName(1,'EL');
MPS_ori.cloneAndAppend(MP_2);

% Bodyscale
if MBS_ori.getSize() > 1
    for i = 1 : MBS_ori.getSize()
        MBS_ori.remove(MBS_ori.get(0));
    end
end
% MBS_ori.get(0).setName('metacarpal2');
% BS_new_1 = MBS_ori.get(0).clone();
% BS_new_1.setName('metacarpal3');
% MBS_ori.cloneAndAppend(BS_new_1);
% BS_new_2 = MBS_ori.get(0).clone();
% BS_new_2.setName('metacarpal4');
% MBS_ori.cloneAndAppend(BS_new_2);
% BS_new_3 = MBS_ori.get(0).clone();
% BS_new_3.setName('metacarpal5');
% MBS_ori.cloneAndAppend(BS_new_3);

% Measurement_Set.cloneAndAppend(MS);

%% Scale Set Setting

Scale_Set = Model_scaler.getScaleSet();
Scale_Set.setName('changed ScaleSet');

if Scale_Set.getSize() > 1
    for i = 1 : Scale_Set.getSize()-1
        Scale_Set.remove(1);
    end
end

Scale_Set.get(0).setSegmentName('auxprfem');
Scale_Set.get(0).setScaleFactors(Vec3(0.9,0.9,0.9));
% 
% 
SS = Scale_Set.get(0).clone();
SS.setSegmentName('auxprrud');
Scale_Set.cloneAndAppend(SS);
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('metacarpal3');
% Scale_Set.cloneAndAppend(SS);

% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('metacarpal4');
% Scale_Set.cloneAndAppend(SS);
% % 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('auxdrrud');
% SS.setScaleFactors(Vec3(1.2,1.2,1.2));
% Scale_Set.cloneAndAppend(SS);

% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('capitate');
% SS.setScaleFactors(Vec3(1.2,1.2,1.2));
% Scale_Set.cloneAndAppend(SS);
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('hamate');
% SS.setScaleFactors(Vec3(1.2,1.2,1.2));
% Scale_Set.cloneAndAppend(SS);

% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('ulna');
% Scale_Set.cloneAndAppend(SS);
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('radius');
% Scale_Set.cloneAndAppend(SS);
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('Proximalphalanx1');
% Scale_Set.cloneAndAppend(SS);
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('Proximalphalanx2');
% Scale_Set.cloneAndAppend(SS);
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('Proximalphalanx3');
% Scale_Set.cloneAndAppend(SS);
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('Proximalphalanx4');
% Scale_Set.cloneAndAppend(SS);
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('Proximalphalanx5');
% Scale_Set.cloneAndAppend(SS);
% 
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('Medialphalanx2');
% Scale_Set.cloneAndAppend(SS);
% 
% SS = Scale_Set.get(0).clone();
% SS.setSegmentName('Medialphalanx3');
% Scale_Set.cloneAndAppend(SS);

% Model_scaler.setMarkerFileName();
% Model_scaler.setMeasurementSet();

%% Marker_Placer setting
Marker_placer = Scale_file.getMarkerPlacer();
Marker_placer.setMarkerFileName(StaticName);
Marker_placer.getTimeRange().set(0,0);
Marker_placer.getTimeRange().set(1,maximal_time);
% Marker_placer.setOutputMarkerFileName('Output_Marker.xml');
Marker_placer.setOutputModelFileName(OutputMarkerModelName);
Marker_placer.setOutputMotionFileName(outputmotion_name);

IKTS = Marker_placer.getIKTaskSet();
IKTS.setName('changed_IKTaskSet');
IKMT = IKTS.get(0); % template for IKTask
IKCT = IKTS.get(40).clone();
for i = 1:IKTS.getSize()
    IKTS.remove(1); % keep the first IKTask with index 0 (for using cloneAndAppend() without crashing)
end

MarkerList = {'EM','EL','RS','US','CMC1','CMC2','CMC5','MP1','MP2','MP3','MP4','MP5',...
              'IP','PP2','PP3','PP4','PP5','DP2','DP3','DP4','DP5','FT1','FT2','FT3','FT4','FT5','PIP2'};
         
          
          
for i = 1 : length(MarkerList)  
    IKMT_new = IKMT.clone();
    IKMT_new.setName(string(MarkerList{i}));
    IKMT_new.setWeight(1);
    IKTS.cloneAndAppend(IKMT_new);
end


% WeightMarkerList = ["MCP3","PP3","DP3","FT3"];
% 
% for i = 1 : length(WeightMarkerList)  
%     IKTS.get(WeightMarkerList(i)).setWeight(1000);
% end

% IKCoordinatelist = ["MCP2_coord_1","PIP2","DIP2",...
%                     "MCP3_coord_1","PIP3","DIP3",...
%                     "MCP4_coord_1","PIP4","DIP4",...
%                     "MCP5_coord_1","PIP5","DIP5"];
IKCoordinatelist = ["PIP2","DIP2",...
                    "PIP3","DIP3",...
                    "PIP4","DIP4",...
                    "PIP5","DIP5"];
for i = 1 : length(IKCoordinatelist)  
    IKCT_new = IKCT.clone();
    IKCT_new.setName(string(IKCoordinatelist(i)));
    IKCT_new.setWeight(1);
    IKTS.cloneAndAppend(IKCT_new);
end

IKTS.remove(0); % remove the old IKTask with index 0 (for using cloneAndAppend() without crashing)
Scale_file.print(ScaleFileName);


