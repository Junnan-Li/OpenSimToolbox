% iterative inverse kinematic calculation   
% 
% input:
%       x_d: [pos;eulerxyz]
%       varargin:   1: iter_max
%                   2: tol
% Junnan Li


function [q,x_res,phi_x,iter] = ik_numeric(om, coord_list, mp_index, x_d, varargin)

assert(length(om.marker_point_list)>= mp_index, 'ik_numeric: no such a marker_point')
assert(length(x_d)== 6, 'ik_numeric: incorrect dimension of xd')

if nargin == 4
    iter_max = 100;
    tol = [1e-3*ones(3,1);1e-2*ones(3,1);];
    alpha = 0.1;
elseif nargin == 5
    iter_max = varargin{1};
    tol = [1e-4*ones(3,1);1e-2*ones(3,1);];
    alpha = 0.1;
elseif nargin == 6
    iter_max = varargin{1};
    tol = varargin{2};
    alpha = 0.1;
elseif nargin == 7
    iter_max = varargin{1};
    tol = varargin{2};
    alpha = varargin{3};
end

q_value = om.get_coordinate_value(coord_list);


for i = 1:iter_max
    om.set_coordinate_value(coord_list, q_value);
    q_value = om.get_coordinate_value(coord_list);
    x_p_i = om.get_mp_frame(mp_index);
    % visualization
%     om.plot_all_body;
    om.plot_mp_frame
    om.model_visualize;

    % update 
    delta_x_i = x_d - x_p_i;
    if isempty(find(abs(delta_x_i)-tol>0))
        disp('ik_numeric: ik finished!')
        break
    end
    J = om.getJacobian_point_sub_ana(mp_index,coord_list );
%     J = om.getJacobian_point_sub(mp_index,coord_list );
    J_inv = pinv(J);
    if rank(J) < min(size(J)) | rank(J_inv) < min(size(J_inv))
        disp('ik_numeric: Jacobian rank deficit')
    end

    delta_q = alpha * J_inv * delta_x_i;
%     max(delta_q)
%     max(delta_x_i)
%     delta_q_sar = 0.1/max(abs(delta_q)) * delta_q;
    q_value = q_value + delta_q; 

end

x_res = x_p_i;
phi_x = delta_x_i;
q = q_value;
iter = i;
end








