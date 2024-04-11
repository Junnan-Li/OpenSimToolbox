function List_Name = Setlist_read(Set)
%SETLIST_READ Summary of this function goes here
%   Detailed explanation goes here
import org.opensim.modeling.*
List_Name = [];
if  Set.getSize() <= 0
%     fprintf('%s List is empty. Unable to load. \n', Set.getClassName);
    return
end

for i = 0 : Set.getSize() - 1
    List_Name = [List_Name; {char(Set.get(i))}];
end
end

