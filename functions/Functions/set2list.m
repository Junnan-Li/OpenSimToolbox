function set_list = set2list(set)

import org.opensim.modeling.*


if  set.getSize() <= 0
    fprintf('%s List is empty. Unable to load. \n', set.getClassName);
    return
end
set_list = cell(set.getSize(),1);
for i = 0 : set.getSize() - 1
    set_list{i+1} = char(set.get(i));
end
end