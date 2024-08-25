%   Calculate the maximal Cartesian acceleration of the markerpoint
% 
% input:
%       model: 
%       mp_index:       index of the mp in the model
%       acc_direction_opt:  
%                           1: 6 directions along axes



function  [x_sol,acc] = cal_max_acc_subdirection(model, opt)


% Jacobian 
J = model.getJacobian_mp_minimal(opt.mp_index);
% Mass matrix
M = model.get_MassMatrix_minimal;
% Moment arm matrix 
MA = model.get_MomentArmMatrix_minimal_AllMus;

% Maximal Isomatrix force as diagonal matrix
F_MIF = model.get_MaxIsometricForce_all_diag;
% Force Length Multiplier as diagonal matrix
mus_FLM = model.get_FiberForceLengthMultiplier;
% get Passive force vector
F_P = model.get_PassiveFiberForce();
mus_PLM = model.get_PassiveForceLengthMultiplier;
alpha = model.get_PennationAngle;

% optimization settings
x0 = rand(length(model.MuscleSet_list),1);

lb = 0.001 *ones(length(model.MuscleSet_list),1);
ub = ones(length(model.MuscleSet_list),1);
nonlcon = [];



switch opt.acc_direction_opt
    case 1 
        x_sol = zeros(length(model.MuscleSet_list),6);
        acc = zeros(6,6);
        tic
        for i = 1:3 % x, y, z axis
            for j = 1:2 % + and - along each axis
                Dir_Sel_vsc = zeros(1,6);
                Dir_Sel_vsc(i) = sign(j-1.5)*1     
                index = [1:6]; % in equality constraints consider other dimensions
                index(i) = []
                Aeq = J(index,:)*inv(M)*MA*F_MIF * diag(mus_FLM) ;
                beq = -J(index,:)*inv(M)*MA*F_P;
                % function of calculating acceleration
                if strcmp(opt.method,'fmincon')
                    fun = @(x) Dir_Sel_vsc*J*inv(M)*MA*((F_MIF * diag(mus_FLM) * x + F_P) .*cos(alpha));
                    A = [];
                    b = [];
                    options = optimoptions('fmincon','Algorithm','sqp','Display','iter-detailed');
                    x_opt = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

                elseif strcmp(opt.method,'lsqlin')
                    % to be corrected. Currently calculating the minimal
                    C = Dir_Sel_vsc*J*inv(M)*MA*F_MIF * diag(mus_FLM);
                    d = Dir_Sel_vsc*J*inv(M)*MA*F_P;
%                     C = J*inv(M)*MA*F_MIF * mus_FLM;
%                     d = J*inv(M)*MA*F_P;
                    A = [];
                    b = [];
                    options = optimoptions('lsqlin','Algorithm','active-set','Display','iter-detailed');
                    [x_opt,resnorm,residual,exitflag,output,lambda] = lsqlin(C,d,A,b,Aeq,beq,lb,ub,x0,options);
                end
                x_sol(:,2*i+j-2) = x_opt;
                acc(1:6,2*i+j-2) = J*inv(M)*MA*(F_MIF * diag(mus_FLM) * x_opt + F_P);
%                 x_sol(:,1) = x_opt;
%                 acc(1:6,1) = J*inv(M)*MA*(F_MIF * mus_FLM .* x_opt + F_P);
%                 f = pinv(J')*MA*(F_MIF * mus_FLM * x_sol(:,i) + F_P);
            end
        end
         t = toc;

end


end