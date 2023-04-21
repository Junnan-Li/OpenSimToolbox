function muscle_properties = update_muscle_properties_in_row(model,state,muscle_properties,row)
%

mus_ori = model.getMuscles();

fns = fieldnames(muscle_properties);
for i = 1:length(fns)
   muscle_properties.(fns{i}){row,1} = state.getTime;
end

for j = 1:mus_ori.getSize
        muscle_properties.Excitation{row,j+1} = mus_ori.get(j-1).getExcitation(state);
        muscle_properties.FiberForce{row,j+1} = mus_ori.get(j-1).getFiberForce(state);
        muscle_properties.FiberForcealoneTendon{row,j+1} = mus_ori.get(j-1).getFiberForceAlongTendon(state);
        muscle_properties.Actuation{row,j+1} = mus_ori.get(j-1).getActuation(state);
end

end

