function [ControlLinear] = RRA_CCL_set(ControlLinear, ParameterMax,ParameterMin, Kp, Kv)
%SETLIST_READ Summary of this function goes here
%   Detailed explanation goes here
import org.opensim.modeling.*


ControlLinear.setIsModelControl(true);
ControlLinear.setExtrapolate(true);
ControlLinear.setDefaultParameterMin(ParameterMin);
ControlLinear.setDefaultParameterMax(ParameterMax);
ControlLinear.setFilterOn(false);
ControlLinear.setUseSteps(false);
ControlLinear.setKp(Kp);
ControlLinear.setKv(Kv);

end