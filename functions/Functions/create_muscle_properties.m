function muscle_properties = create_muscle_properties(model)
%
muscle_properties = struct;

muscle_properties.Excitation = {};
muscle_properties.FiberForce = {};
muscle_properties.FiberForcealoneTendon = {};
muscle_properties.Actuation = {};

mus_ori = model.getMuscles();
fns = fieldnames(muscle_properties);

muscle_name_row = {};
muscle_name_row{1,1} = char('Time');
for j = 1: mus_ori.getSize
    muscle_name_row{1,j+1} = char(mus_ori.get(j-1).getName.toString);
end

for i = 1:length(fns)
   muscle_properties.(fns{i}) = muscle_name_row;
end


end

