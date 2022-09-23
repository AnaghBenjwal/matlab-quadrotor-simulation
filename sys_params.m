global params;

params.m = 0.2;
params.g = 9.81;
params.Ix = 5e-3;
params.Iy = 5e-3;
params.Iz = 8e-3;

params.I = [params.Ix 0 0; 0 params.Iy 0; 0 0 params.Iz];

params.l = 0.086;
params.Km = 1.5e-9;
params.Kf = 6.11e-8;

%initial state
global state;

state.pos = [0; 0; 0];
state.vel = [0; 0; 0];
state.acc = [0; 0; 0];

state.rot = [0; 0; 0];
state.rot_d = [0; 0; 0];
state.omega = [0; 0; 0];
state.alpha = [0; 0; 0];

%desired state
global des_state;
des_state.z = 10;
des_state.pos = [0; 0; des_state.z];
des_state.vel = [0; 0; 0];
des_state.acc = [0; 0; 0];

des_state.psi = 0;
des_state.rot = [0; 0; des_state.psi];
des_state.rot_d = [0; 0; 0];

global time;
%simulation_time
time.start_time = 0;
time.time_step = 0.1;
time.end_time = 50;

