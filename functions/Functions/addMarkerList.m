function Model = addMarkerList(Model,MarkerName, MarkerFrame, MarkerLocation, fix)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin == 4
    fix = false;
end
length_Name = length(MarkerName);
length_Frame = length(MarkerFrame);
length_Location = length(MarkerLocation);

if (length_Name~=length_Frame) || (length_Name~= length_Location)
    fprintf('Marker input size not correct. \n');
    return
end
import org.opensim.modeling.*

for i = 1:length_Name
    marker = Marker( MarkerName{i}, Model.getBodySet().get(MarkerFrame{i}), MarkerLocation{i});
    %     marker = Marker( MarkerName{i}, MarkerFrame{i}, MarkerLocation{i});
    marker.set_fixed(fix)
    Model.addMarker(marker);
end





