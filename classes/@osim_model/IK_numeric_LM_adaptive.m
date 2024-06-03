% iterative inverse kinematic calculation   
% 
% input:
%       x_d: [pos;eulerxyz]
%       varargin:   1: iter_max
%                   2: tol
% Junnan Li


function [q,x_res,phi_x,iter] = IK_numeric_LM_adaptive(om, coord_list, mp_index, x_d, W_e, varargin)

assert(length(om.marker_point_list)>= mp_index, 'IK_numeric_LM_adaptive: no such a marker_point')
assert(length(x_d)== 6, 'IK_numeric_LM_adaptive: incorrect dimension of xd')

if nargin == 5
    iter_max = 100;
    tol = [1e-3*ones(3,1);1e-2*ones(3,1);];
elseif nargin == 6
    iter_max = varargin{1};
    tol = [1e-4*ones(3,1);1e-2*ones(3,1);];
elseif nargin == 7
    iter_max = varargin{1};
    tol = varargin{2};
elseif nargin == 8
    iter_max = varargin{1};
    tol = varargin{2};
end

q_value = om.get_coordinate_value(coord_list);
R_d = euler2R_XYZ(x_d(4:6));

for i = 1:iter_max
    om.set_coordinate_value(coord_list, q_value);
    q_value = om.get_coordinate_value(coord_list);
    x_p_i = om.get_mp_frame(mp_index);
    % visualization
%     om.plot_all_body;
    om.plot_mp_frame
%     om.model_visualize;

    % update 

    delta_p_i = x_d(1:3) - x_p_i(1:3);
    R_i = euler2R_XYZ(x_p_i(4:6));
    R_cal = R_d*R_i';
    l = [R_cal(3,2)-R_cal(2,3);R_cal(1,3)-R_cal(3,1);R_cal(2,1)-R_cal(1,2)];
    delta_r_i = (atan2(norm(l),R_cal(1,1)+R_cal(2,2)+R_cal(3,3)-1))/norm(l)*l;
    delta_x_i = [delta_p_i;delta_r_i];


    if isempty(find(abs(delta_x_i)-tol>0))
        disp('IK_numeric_LM_adaptive: ik finished!')
        break
    end
    J = om.getJacobian_mp_sub_ana(mp_index,coord_list );
%     J = om.getJacobian_mp_sub(mp_index,coord_list );
    g_i = J'*W_e*delta_x_i;
    W_d = 1/2*delta_x_i'*W_e*delta_x_i*eye(length(q_value)) + 1e-7*eye(length(q_value));
    delta_q = inv(J'*W_e*J + W_d) * g_i;
    q_value = q_value + delta_q; 

end

x_res = x_p_i;
phi_x = delta_x_i;
q = q_value;
iter = i;
end








