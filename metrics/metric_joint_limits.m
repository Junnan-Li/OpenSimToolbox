%   Penaliyation of the joint limits;
%   parameter is set as the q value in rad
% 
% source:
% [1] T. F. Chan and R. V Dubey, "A Weighted Least-Norm Solution Based Scheme 
% for Avoiding Joint Limits for Redundant Joint Manipulators," IEEE Trans. Robot. Autom., vol. 1, no. 2, 1995.
% [2] N. Vahrenkamp and T. Asfour, “Representing the robot’s workspace through 
% constrained manipulability analysis,” Auton. Robots, vol. 38, no. 1, pp. 17–30, 2015, doi: 10.1007/s10514-014-9394-z.



function  metric_jl = metric_joint_limits(q, q_limit,varargin)

if nargin == 2
    par = 4*180/pi;
else
    par = varargin{1}*180/pi;
end
% q: [nj,n]
nj = size(q,2);
assert(~isempty(find(size(q_limit) ~= [nj 2])),'[penalty_joint_limits]: wrong input dimension!')

q_limits_low = q_limit(:,1);
q_limits_high = q_limit(:,2);

% H = 1/4 * (q_limits_high-q_limits_low)^2 ./ ((q_limits_high - q).*(q-q_limits_low));

Hdot = 1/par*(q_limits_high-q_limits_low).^2 .* (2*q - q_limits_high-q_limits_low) ./ ...
                                        ((q_limits_high - q).^2 .* (q-q_limits_low).^2);

metric_jl = 1./(sqrt(1+abs(Hdot)));


end