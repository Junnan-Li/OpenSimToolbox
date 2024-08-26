%   Calculate the maximal Cartesian acceleration of the markerpoint in all
%   directions, originated from [0,0,0]
% 
% input:
%       model: 
%       mp_index:       index of the mp in the model
%       acc_direction_opt:  
%                           1: 6 directions along axes

% 
% Optimization:
%               x: points on the polytope


function  [x_sol,acc] = cal_max_acc_radius(model, opt)


% Jacobian 
J = model.getJacobian_mp_minimal(opt.mp_index);
% Mass matrix
M = model.get_MassMatrix_minimal;
% Moment arm matrix 
MA = model.get_MomentArmMatrix_minimal_AllMus;

% Maximal Isomatrix force as diagonal matrix
F_MIF = model.get_MaxIsometricForce;
% Force Length Multiplier as diagonal matrix
mus_FLM = model.get_FiberForceLengthMultiplier;
% get Passive force vector
F_P = model.get_PassiveFiberForce();
mus_PLM = model.get_PassiveForceLengthMultiplier;
alpha = model.get_PennationAngle;

% optimization settings
x0 = rand(length(model.MuscleSet_list),1);

lb = 0.1*ones(length(model.MuscleSet_list),1);
ub = ones(length(model.MuscleSet_list),1);
nonlcon = [];

n_mus = size(MA,2);

switch opt.acc_direction_opt
    case 1
        x_sol = zeros(length(model.MuscleSet_list),1);
        acc = zeros(6,1);
%         tic
        Aeq = [eye(n_mus);-eye(n_mus)];
        beq = [];

        % function of calculating acceleration
        if strcmp(opt.method,'fmincon')
        elseif strcmp(opt.method,'lsqlin')
        elseif strcmp(opt.method,'global')
            if opt.only_translational
                fun = @(x) norm(pinv(J(1:3,:)')*MA*(F_MIF.*mus_FLM.*cos(alpha).* x)); % + F_P
            else
            end
            x_opt = zeros(n_mus,n_mus);
            for j = 1:n_mus
                lb = 0.1*ones(length(model.MuscleSet_list),1);
                lb(j) = 1;
                problem = createOptimProblem('fmincon',...
                    'objective',fun,...
                    'x0',x0,...
                    'lb',lb,'ub',ub,...
                    'options',optimoptions(@fmincon,'Algorithm','sqp','Display','off','ConstraintTolerance',1e-6));
                gs = GlobalSearch;
                [x_opt(:,j),~] = run(gs,problem);
            end
        end
        x_sol(:,1) = x_opt;
        acc(1:6,1) = J*inv(M)*MA*(F_MIF .* (mus_FLM .* x_opt + mus_PLM) .* cos(alpha)); %  + F_P
%         t = toc;

end


end