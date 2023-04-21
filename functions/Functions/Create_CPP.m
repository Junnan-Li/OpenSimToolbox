function CPP = Create_CPP(Name, Body, Location, Coordinate, MaxRange, MinRange)


if nargin == 5; MinRange = 0;end
if nargin == 4; MaxRange = 0; MinRange = 0;end
if nargin < 4
    fprintf('ConditionalPathPoint input for %s are not enough. \n', Name);
    return
end

import org.opensim.modeling.*

if ~strcmp(Body.getClassName, 'Body') || ...
   ~strcmp(Coordinate.getClassName, 'Coordinate') 
    fprintf('Input Variables Format for %s are wrong. \n', Name);
    return
end



CPP = ConditionalPathPoint();
CPP.setName(Name);
CPP.setBody(Body);
CPP.set_location(Location);
CPP.setCoordinate(Coordinate);
CPP.setRangeMax(MaxRange);
CPP.setRangeMin(MinRange);

% PathPointSet.adoptAndAppend(CPP);
end

