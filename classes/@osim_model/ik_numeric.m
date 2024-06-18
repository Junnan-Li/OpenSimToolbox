% iterative inverse kinematic calculation   
% 
% input:
%       x_d: [pos;eulerxyz]
%       varargin:   1: iter_max
%                   2: tol
% Junnan Li


function [q,info] = IK_numeric(om, coord_list, mp_index, x_d, varargin)

assert(length(om.marker_point_list)>= mp_index, 'ik_numeric: no such a marker_point')
assert(length(x_d)== 6, 'ik_numeric: incorrect dimension of xd')

if nargin == 4
    par = om.IK_numeric_par();
elseif nargin == 5
    par = varargin{1};
else
    error('[IK_numeric]: input dimension is incorrect! ')
end

iter_max = par.iter_max;
tol = par.tol;
alpha = par.alpha;
retry_num = par.retry_num;

q_value = om.get_coordinate_value(coord_list);
info = struct();
info.status = 0;
info.x_res = zeros(size(x_d));
info.phi_x = zeros(size(x_d));
info.iter = 0;

for retry_i = 0:retry_num
    try
        for i = 1:iter_max
            om.set_coordinate_value(coord_list, q_value);
            q_value = om.get_coordinate_value(coord_list);
            x_p_i = om.get_mp_frame(mp_index);
            if par.visual % visualization
                %     om.plot_all_body;
                om.plot_mp_frame
                %     om.model_visualize;
            end

            % update
            phi_p_i = x_d(1:3) - x_p_i(1:3);
            phi_R_i = euler2R_XYZ(x_d(4:6)) * euler2R_XYZ(x_p_i(4:6))';
            phi_R_i_eul = R2euler_XYZ(phi_R_i);
            %     delta_x_i = x_d - x_p_i;
            delta_x_i = [phi_p_i;phi_R_i_eul];

            if isempty(find(abs(delta_x_i)-tol>0))
                disp('ik_numeric: ik finished!')
                info.status = 1;
                break
            end
            J = om.getJacobian_mp_sub_ana(mp_index,coord_list );
            %     J = om.getJacobian_point_sub(mp_index,coord_list );
            J_inv = pinv(J);
            if rank(J) < min(size(J)) | rank(J_inv) < min(size(J_inv))
                disp('ik_numeric: Jacobian rank deficit')
            end
            delta_q = alpha * J_inv * delta_x_i;
            q_value = q_value + delta_q;
        end
    catch ME
        q_value = rand(size(q_value));
    end
    if info.status % IK solved
        break
    end
end
info.retry_iter = retry_i;
info.x_res = x_p_i;
info.phi_x = delta_x_i;
q = q_value;
info.iter = i;
end








