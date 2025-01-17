% IK parameter template for IK_numerix_LM function 
% 
% Junnan Li


function par_Struct = IK_numeric_LM_par(om, num_q)

par_Struct = struct();
par_Struct.iter_max = 100;
par_Struct.retry_num = 10;
par_Struct.tol = [1e-3*ones(3,1);1e-2*ones(3,1)];
par_Struct.W_e = diag([1,1,1,1,1,1]);
par_Struct.W_d = 1e-4*diag(ones(num_q,1));
par_Struct.visual = 0;

end