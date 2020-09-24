% Author: Armando Alvarez Rolins
% Master's Thesis
% Aug. 29th, 2016
% Title: ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method


function totalAccel = RotDockingCost_Mix(r_w,v_w,tof_w)


% Universal Conditions
global tof t_go1 t_go2 dockLength rotRate_target angVel_target r_target v_target r_waypoint v_waypoint tof_waypoint
r_waypoint = r_w;
v_waypoint = v_w;
tof_waypoint = tof_w;
tof = 150; % [seconds]
time_array_waypoint = linspace(0,tof_waypoint,1000);
t_go1 = tof_waypoint;
t_go2 = tof-tof_waypoint;

% Target Conditions
theta_target = 0; % [rad]
dockLength = 0.4; % [m]
rotRate_target = 1; % [rpm]
angVel_target = rotRate_target*2*pi/60; % [rad/s]
r_target = dockLength*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m]
v_target = angVel_target*norm(r_target)*[cos(angVel_target*tof +pi);sin(angVel_target*tof +pi)]; % [m/s]

% Chaser Conditions
r_chaser(:,1) = [2.5 ; 1.3];
v_chaser(:,1) = [0; 0];
a_chaser(:,1) = [0; 0];
state_array0 = [r_chaser(1,1); r_chaser(2,1);
    v_chaser(1,1); v_chaser(2,1); a_chaser(1,1); a_chaser(2,1)];

% Time Propagation until waypoint
[t,state_array] = ode113('Mix_waypoint',time_array_waypoint,state_array0);
v_chaser_leg1 = [state_array(:,3) state_array(:,4)]';
r_chaser_leg1 = [state_array(:,1) state_array(:,2)]';
a_chaser_leg1 = [state_array(:,5) state_array(:,6)]';
for i = 1:numel(a_chaser_leg1(:,1))
   magAccel_leg1 = norm([a_chaser_leg1(i,1); a_chaser_leg1(i,2)]); 
end

% Time Propagation until target
state_array0 = [r_chaser_leg1(1,end); r_chaser_leg1(2,end);
    v_chaser_leg1(1,end-1); v_chaser_leg1(2,end-1); a_chaser(1,end); a_chaser(2,end)];
tof = tof-tof_waypoint;
time_array_final = linspace(0,tof,1000);
[t,state_array] = ode113('Mix',time_array_final,state_array0);
v_chaser_leg2 = [state_array(:,3) state_array(:,4)]';
r_chaser_leg2 = [state_array(:,1) state_array(:,2)]';
a_chaser_leg2 = [state_array(:,5) state_array(:,6)]';
for i = 1:numel(a_chaser_leg2(:,1))
   magAccel_leg2 = norm([a_chaser_leg2(i,1) a_chaser_leg2(i,2)]); 
end

% Total Acceleration for Rendezvous
totalAccel_leg1 = sum(magAccel_leg1);
totalAccel_leg2 = sum(magAccel_leg2);
totalAccel = totalAccel_leg1+totalAccel_leg2;

figure(1)
hold on
radius = 0.4;
th = 0:pi/50:2*pi;
xunit = radius * cos(th);
yunit = radius * sin(th);
h = plot(xunit, yunit);
grid on
plot(r_chaser_leg1(1,:),r_chaser_leg1(2,:))
plot(r_chaser_leg2(1,:),r_chaser_leg2(2,:))
plot([0;r_target(1,1)],[0;r_target(2,1)])
end