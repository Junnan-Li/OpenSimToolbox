%   Calculate the maximal Cartesian acceleration of the markerpoint
% 
% input:
%       model: 
%       mp_index:       index of the mp in the model
%       acc_direction_opt:  
%                           1: 6 directions along axes



function  [x_sol,acc] = cal_max_acc(model, opt)


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
mus_PLM(20) = 0; % disa
alpha = model.get_PennationAngle;

% optimization settings
x0 = rand(length(model.MuscleSet_list),1);

lb = 0.1*ones(length(model.MuscleSet_list),1);
ub = ones(length(model.MuscleSet_list),1);
nonlcon = [];



switch opt.acc_direction_opt
    case 1
        x_sol = zeros(length(model.MuscleSet_list),1);
        acc = zeros(6,1);
        Aeq = [];
        beq = [];
        % function of calculating acceleration
        if strcmp(opt.method,'fmincon')
            fun = @(x) -norm(J*inv(M)*MA*((F_MIF.*mus_FLM.*x+mus_PLM).*cos(alpha)));
            problem = createOptimProblem('fmincon',...
                'objective',fun,...
                'x0',x0,...
                'lb',lb,'ub',ub,'Aeq',Aeq,'beq',beq,...
                'options',optimoptions(@fmincon,'Algorithm','sqp','Display','off'));
            gs = GlobalSearch;
            [x_opt,~] = run(gs,problem);
        elseif strcmp(opt.method,'lsqlin')
            C = J*inv(M)*MA*F_MIF * mus_FLM;
            d = J*inv(M)*MA*F_P;
            A = [];
            b = [];
            options = optimoptions('lsqlin','Algorithm','active-set','Display','iter-detailed');
            x_opt = lsqlin(C,d,A,b,Aeq,beq,lb,ub,x0,options);
        end
        x_sol(:,1) = x_opt;
        acc(1:6,1) = J*inv(M)*MA*(F_MIF.*(mus_FLM.*x_opt+mus_PLM).*cos(alpha)); %  + F_P
end


end