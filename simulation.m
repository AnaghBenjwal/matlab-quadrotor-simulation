sys_params;

time_steps = time.start_time:time.time_step:time.end_time;
n = numel(time_steps);
count = 0;
z_plot = zeros(1, n);
vel_plot = zeros(1, n);
acc_plot = zeros(1, n);
%psi_plot = zeros(1, n);
%omega_plot = zeros(1, n);
%alpha_plot = zeros(1, n);

for t = time.start_time:time.time_step:time.end_time
    
    inputs = controller(params, state, des_state, time);
   
    count = count + 1;
    z_plot(count) = state.pos(3);
    vel_plot(count) = state.vel(3);
    acc_plot(count) = state.acc(3);
    %psi_plot(count) = state.rot(3);
    %omega_plot(count) = state.omega(3);
    %alpha_plot(count) = state.alpha(3);
    
    state.omega = omega_from_rotd(state);
    state.acc = acceleration(inputs, params ,state);
    state.alpha = angular_acceleration(inputs, params, state);
    
    state.omega = state.omega + (time.time_step * state.alpha);
    state.rot_d = rotd_from_omega(state);
    state.rot = state.rot + (time.time_step * state.rot_d);
    state.vel = state.vel + (time.time_step * state.acc);
    state.pos = state.pos + (time.time_step * state.vel);
    
end

function tau = torque(inputs, params)

tau = [params.l * params.Km * (inputs(4) - inputs(2));
    params.l * params.Km * (inputs(3) - inputs(1));
    params.Kf * (inputs(4) - inputs(3) + inputs(2) - inputs(1))];

end

function T = thrust(inputs, params)

T = [0; 0; params.Kf * sum(inputs)];

end

function acc = acceleration(inputs, params, state)

R = rotation(state.rot);
T = R * thrust(inputs, params);
acc = [0; 0; -params.g] + (1 / params.m) * T;

end

function alpha = angular_acceleration(inputs, params, state)

tau = torque(inputs, params);
alpha = inv(params.I) * (tau - cross(state.omega, params.I * state.omega));

end

function omega = omega_from_rotd(state)
%converts derivatives of pitch, roll, and yaw angles to angular velocities
R = [1, 0, -sin(state.rot(2));
    0, cos(state.rot(1)), cos(state.rot(2))*sin(state.rot(1));
    0, -sin(state.rot(1)), cos(state.rot(2))*cos(state.rot(1))];

omega = R * state.rot_d;

end

function rot_d = rotd_from_omega(state)
%convert angular velocities to derivatives of pitch, roll, and yaw angles

R = [1, 0, -sin(state.rot(2));
    0, cos(state.rot(1)), cos(state.rot(2))*sin(state.rot(1));
    0, -sin(state.rot(1)), cos(state.rot(2))*cos(state.rot(1))];

rot_d = inv(R) * state.omega;

end
