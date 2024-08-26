

function opt = init_metric_opt()

opt = struct();
opt.method = 'fmincon';
opt.mp_index = 1;
opt.acc_direction_opt = 1;
opt.only_translational = 0;




end