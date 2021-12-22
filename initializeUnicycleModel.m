% initializeUnicycleModel.m
% 
% author: Beniamino Pozzan
%
% date: 7/12/2021
%
%
% This script initialize the variables required by the unicycle generated
% by unicycle_posture_template.slx .
% The variables are stored in the structure "unicycle"


clear unicycle

% differential drive parameters
unicycle.diffDrive.r = 0.02;    % wheel radius [m]
unicycle.diffDrive.d = 0.2;     % wheel distance [m]
unicycle.diffDrive.Td = 0.01;   % l.p.f. time constant [s]
unicycle.diffDrive.omegaWheelMax = 20;   % max wheels speed [rad/s]
unicycle.diffDrive.omegaWheelMin = -20;  % min wheels speed [rad/s]


% initial condition
unicycle.ic.x = 0;       % initial x coordinate [m]
unicycle.ic.y = 0;       % initial y coordinate [m]
unicycle.ic.theta = 0;   % initial heading angle [rad]