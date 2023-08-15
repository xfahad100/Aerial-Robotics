function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
mass = params.mass;
g = params.gravity;
rdot3 = state.vel(3);
r3 = state.pos(3);
phiT = des_state.yaw; phides = des_state.yaw;
kd3 = 20; kp3 = 100;
%position controller
cmd_acc = des_state.acc + kd3.*(des_state.vel - state.vel) + kp3.*(des_state.pos - state.pos);
u1 = mass*g + mass*cmd_acc(3);
phic = ((cmd_acc(1)*sin(phiT) - cmd_acc(2)*cos(phiT)))/g;
thetac = ((cmd_acc(1)*cos(phiT) + cmd_acc(2)*sin(phiT)))/g;
Rc = [phic, thetac, phides];

%attitude controller
kp_phi = 500; kd_phi = 10;
kq_theta = 500; kd_theta = 10;
kr_psi = 500; kd_psi = 10;
u2 = [kp_phi.*(Rc(1) - state.rot(1)) + kd_phi.*(-state.omega(1));
     kq_theta.*(Rc(2) - state.rot(2)) + kd_theta.*(-state.omega(2));
     kr_psi.*(Rc(3) - state.rot(3)) + kd_psi.*(des_state.yawdot - state.omega(3))];

% Thrust
F = u1;

% Moment
M = u2;
%M = zeros(3,1);

% =================== Your code ends here ===================

end
