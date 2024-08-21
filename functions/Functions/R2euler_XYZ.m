function angle = R2euler_XYZ(R)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

alpha = atan(-R(2,3)/R(3,3));
beta = asin(R(1,3));
gamma = atan(-R(1,2)/R(1,1));


if abs(cos(beta))>1e-9
    alpha = atan2(-R(2,3),R(3,3));
    gamma = atan2(-R(1,2),R(1,1));
end



angle = [alpha;beta;gamma];

end

