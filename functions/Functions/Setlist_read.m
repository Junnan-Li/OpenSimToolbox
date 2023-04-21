function [List_Name] = Setlist_read(List, List_Name)
%SETLIST_READ Summary of this function goes here
%   Detailed explanation goes here
import org.opensim.modeling.*

if  List.getSize() <= 0
    fprintf('%s List is empty. Unable to load. \n', List.getClassName);
    return
end

for i = 0 : List.getSize() - 1
    List_Name = [List_Name; {char(List.get(i))}];
end
end

