% initializeUnicycleModel.m
% 
% author: Beniamino Pozzan
%
% date: 7/12/2021
%
%
% This script initialize the variables required by the quadrotor generated
% by quadrotor_posture_template.slx .
% The variables are stored in the structure "quadrotor" and "simPar"


clear quadrotor simPar


%%% simulation params
simPar.g = 9.81;    % gravity value


%%% GEOMETRY

% rotors' relative postition w.r.t. base link [m]
% +x : front, +y: right
quadrotor.geometry.p_mot = ...
    [0.13 -0.22 0;
    -0.13 0.2 0;
    0.13 0.22 0;
    -0.13 -0.2 0];
% mass of the base link
quadrotor.geometry.mass = 1.5; % [Kg]
% inertia matrix
quadrotor.geometry.I = diag([0.029125 0.029125 0.055225]); %[Kg*m^2]


%%% MOTORS + ROTORS
% rotors' maximum spinning rate
quadrotor.rotor.maxRotVelocity = 1100; % [rad/s]

% trust and tourque coefficients
quadrotor.rotor.c_f = 5.84e-06; % [N/s^2]
quadrotor.rotor.c_tau = quadrotor.rotor.c_f * 0.06; % [Nm/s^2]
quadrotor.rotor.direction = [-1 -1 1 1]; % 1: CW , -1: CCW


%%% MIXER

% matrices
% from input to thrusts
quadrotor.mixer.F = ...
    [zeros(2,4); quadrotor.rotor.c_f*ones(1,4)];
% from intput to moments
quadrotor.mixer.M = ...
    cross(quadrotor.geometry.p_mot', ...
    kron([0;0;quadrotor.rotor.c_f ],ones(1,4))) ...
    + quadrotor.rotor.direction .* quadrotor.rotor.c_tau .* [0;0;1];

% WRENCH MAPPER

% input combination that gives only thrust
u_thrust = ker(quadrotor.mixer.M);
u_thrust = u_thrust ./ norm(quadrotor.mixer.F*u_thrust);
if([0 0 1]*quadrotor.mixer.F*u_thrust <0)
    u_thrust = -u_thrust;
end

% input combination that gives only torque on roll
u_roll = ints(invt(quadrotor.mixer.M,[1;0;0]),ortco(u_thrust));
u_roll = u_roll ./ norm(quadrotor.mixer.M*u_roll);
if([1 0 0]*quadrotor.mixer.M*u_roll <0)
    u_roll = -u_roll;
end

% input combination that gives only torque on pitch
u_pitch = ints(invt(quadrotor.mixer.M,[0;1;0]),ortco(u_thrust));
u_pitch = u_pitch ./ norm(quadrotor.mixer.M*u_pitch);
if([0 1 0]*quadrotor.mixer.M*u_pitch <0)
    u_pitch = -u_pitch;
end

% input combination that gives only torque on yaw
u_yaw = ints(invt(quadrotor.mixer.M,[0;0;1]),ortco(u_thrust));
u_yaw = u_yaw ./ norm(quadrotor.mixer.M*u_yaw);
if([0 0 1]*quadrotor.mixer.M*u_yaw <0)
    u_yaw = -u_yaw;
end

% from tau_x, tau_y, tau_z, T into rotor spinning rate square
quadrotor.mixer.wrenchMapper = [u_roll, u_pitch, u_yaw, u_thrust];
clear u_*