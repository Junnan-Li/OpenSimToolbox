% iterative inverse kinematic calculation   
% 
% input:
%       x_d: [pos;eulerxyz]
%       varargin:   1: iter_max
%                   2: tol
% Junnan Li


function [q,x_res,phi_x,iter] = IK_numeric_LM(om, coord_list, mp_index, x_d, W_e, W_d,varargin)

assert(length(om.marker_point_list)>= mp_index, 'ik_numeric: no such a marker_point')
assert(length(x_d)== 6, 'ik_numeric: incorrect dimension of xd')

if nargin == 6
    iter_max = 100;
    tol = [1e-3*ones(3,1);1e-2*ones(3,1);];
    alpha = 0.1;
elseif nargin == 7
    iter_max = varargin{1};
    tol = [1e-4*ones(3,1);1e-2*ones(3,1);];
    alpha = 0.1;
elseif nargin == 8
    iter_max = varargin{1};
    tol = varargin{2};
    alpha = 0.1;
elseif nargin == 9
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
%     om.model_visualize;

    % update 
    delta_x_i = x_d - x_p_i;
    if isempty(find(abs(delta_x_i)-tol>0))
        disp('IK_numeric_LM: ik finished!')
        break
    end
    J = om.getJacobian_mp_sub_ana(mp_index,coord_list );

%     J_inv = pinv(J);

%     if rank(J) < min(size(J)) | rank(J_inv) < min(size(J_inv))
%         disp('IK_numeric_LM: Jacobian rank deficit')
%     end
    g_i = J'*W_e*delta_x_i;
    delta_q = inv(J'*W_e*J + W_d) * g_i;
    q_value = q_value + delta_q; 

end

x_res = x_p_i;
phi_x = delta_x_i;
q = q_value;
iter = i;
end








