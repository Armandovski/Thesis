% Author: Armando Alvarez Rolins
% Master's Thesis
% Aug. 29th, 2016
% Title: ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method
clc;
clear all;
close all;

% Universal Conditions
global tof dockLength rotRate_target angVel_target r_target v_target r_waypoint v_waypoint
tof = 150;

% Target Conditions
theta_target = 0; % [rad]
dockLength = 0.4; % [m]
rotRate_target = 1; % [rpm]
angVel_target = rotRate_target*2*pi/60; % [rad/s]
r_target = dockLength*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m]
v_target = angVel_target*norm(r_target)*[cos(angVel_target*tof +pi);sin(angVel_target*tof +pi)]; % [m/s]

% Waypoint Conditions
r_waypoint = 1.3*dockLength*[cos(angVel_target*tof);sin(angVel_target*tof)]; % [m]
v_waypoint = angVel_target*norm(r_waypoint)*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m/s]

tof_w = 0.9*tof;
a = RotDockingCost_LQR(r_waypoint,v_waypoint,tof_w);