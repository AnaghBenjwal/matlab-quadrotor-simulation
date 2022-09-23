function [inputs] = controller(params, state, des_state, time)

%CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

persistent integral_rot
persistent integral_pos

if isempty(integral_rot)
    integral_rot = zeros(3, 1);
end

if isempty(integral_pos)
    integral_pos = zeros(3, 1);
end

if max(abs(integral_rot)) > 0.5
    integral_rot = zeros(3, 1);
end

if max(abs(integral_pos)) > 0.5
    integral_pos = zeros(3, 1);
end

error_rot = des_state.rot - state.rot;
error_rotd = des_state.rot_d - state.rot_d;
error_pos = des_state.pos - state.pos;
error_vel = des_state.vel - state.vel;

Kp = 2;
Kd = 1.9;
Ki = 0.3;

T = ((params.g + (Kp * error_pos(3)) + (Kd * error_vel(3)) + (Ki * integral_pos(3))) * params.m) / cos(state.rot(1))*cos(state.rot(2));
integral_pos = integral_pos + (error_pos * time.time_step);

Kpr = 2;
Kdr = 1.9;
Kir = 0.3;

tau_phi = (Kpr*error_rot(1)) + (Kdr*error_rotd(1)) + (Kir*integral_rot(1)) * params.Ix;
tau_theta = (Kpr*error_rot(2)) + (Kdr*error_rotd(2)) + (Kir*integral_rot(2)) * params.Iy;
tau_psi = (Kpr*error_rot(3)) + (Kdr*error_rotd(3)) + (Kir*integral_rot(3)) * params.Iz;
integral_rot = integral_rot + (error_rot .* time.time_step);

inputs(1) = T / (4 * params.Kf) - tau_theta / (2 * params.Kf * params.l) - tau_psi / (4 * params.Km);
inputs(2) = T / (4 * params.Kf) - tau_phi / (2 * params.Kf * params.l) - tau_psi / (4 * params.Km);
inputs(3) = T / (4 * params.Kf) - tau_theta / (2 * params.Kf * params.l) - tau_psi / (4 * params.Km);
inputs(4) = T / (4 * params.Kf) - tau_phi / (2 * params.Kf * params.l) - tau_psi / (4 * params.Km); 

end


