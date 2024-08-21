% test joint limits penalization functions



% clear all
% clc,


%% test joint limit metric

% [1] T. F. Chan and R. V Dubey, "A Weighted Least-Norm Solution Based Scheme for Avoiding Joint
% Limits for Redundant Joint Manipulators,” IEEE Trans. Robot. Autom., vol. 1, no. 2, 1995.
% [2] N. Vahrenkamp and T. Asfour, "Representing the robot[s workspace through constrained manipulability
% analysis," Auton. Robots, vol. 38, no. 1, pp. 17–30, 2015, doi: 10.1007/s10514-014-9394-z.

q_limit = [0,90;0,80] * pi/180;
q = q_limit(1,1):.02:q_limit(1,2);

[q_1,q_2] = ndgrid(q_limit(1,1):0.001:q_limit(1,2),...
    q_limit(2,1):0.001:q_limit(2,2));

metric_jl_single = metric_joint_limits(q, q_limit(1,:),4)';
metric_jl_i = metric_joint_limits([q_1(:),q_2(:)]', q_limit,4)';
metric_jl = metric_jl_i(:,1).*metric_jl_i(:,2);
% metric_jl_1 = metric_joint_limits(q, [0,90],4)';
C = [1 0 0] .* metric_jl ;

figure(1)
hs = scatter3(q_1(:),q_2(:),metric_jl,10,C);
scatter3(q_1(:),q_2(:),metric_jl,1, [1 0 0].*metric_jl) % 
grid on
axis equal

figure(2)
hs = plot(q(:),metric_jl_single);
xlabel('deg')
ylabel('penalty value')

