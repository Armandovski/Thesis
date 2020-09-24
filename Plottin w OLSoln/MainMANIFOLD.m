% Author: Armando Alvarez Rolins
% Master's Thesis
% Aug. 29th, 2016
% Title: ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method
clc;
clear all;
close all;

% Universal Conditions
global tof dockLength rotRate_target angVel_target r_target v_target r_waypoint v_waypoint
%% LQR
for tof = 150

    % Target Conditions
    theta_target = 0; % [rad]
    dockLength = 0.4; % [m]
    rotRate_target = 1; % [rpm]
    angVel_target = rotRate_target*2*pi/60; % [rad/s]
    r_target = dockLength*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m]
    v_target = angVel_target*norm(r_target)*[cos(angVel_target*tof +pi);sin(angVel_target*tof +pi)]; % [m/s]
    
    % Waypoint Conditions
%     r_waypoint = r_target;
%     v_waypoint = v_target;
%     r_waypoint = 1.3*dockLength*[cos(angVel_target*tof);sin(angVel_target*tof)]; % [m]
%     v_waypoint = angVel_target*norm(r_waypoint)*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m/s]
r_waypoint = [-0.197427067615333;-0.538714572652621;]
v_waypoint = [-0.005;0.001];
    
    tof_w = 0.9*tof;
    
    figure(1)
    hold on
    radius = 0.4;
    th = 0:pi/50:2*pi;
    xunit = radius * cos(th);
    yunit = radius * sin(th);
    title('Least Cost Trajectories at 150 seconds Time of Flight')
    xlabel('x position [m]','FontWeight','bold')
    ylabel('y position [m]','FontWeight','bold')
    [r_chaser_leg1_LQR,r_chaser_leg2_LQR,a_LQR] = RotDockingCostLQR(r_waypoint,v_waypoint,tof_w,tof);

    
    
end

%% Mixed

for tof = 150

    % Target Conditions
    theta_target = 0; % [rad]
    dockLength = 0.4; % [m]
    rotRate_target = 1; % [rpm]
    angVel_target = rotRate_target*2*pi/60; % [rad/s]
    r_target = dockLength*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m]
    v_target = angVel_target*norm(r_target)*[cos(angVel_target*tof +pi);sin(angVel_target*tof +pi)]; % [m/s]
    
    % Waypoint Conditions
%     r_waypoint = r_target;
%     v_waypoint = v_target;
%     r_waypoint = 1.3*dockLength*[cos(angVel_target*tof);sin(angVel_target*tof)]; % [m]
%     v_waypoint = angVel_target*norm(r_waypoint)*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m/s]
r_waypoint = [-0.197427067615333;-0.538714572652621;];
v_waypoint = [-0.005;0.001];
    
    tof_w = 0.9*tof;
    

    [r_chaser_leg1_MIX,r_chaser_leg2_MIX,a_MIX] = RotDockingCostMIX(r_waypoint,v_waypoint,tof_w,tof);

    
    
end

%% Pure ZEM/ZEV
for tof = 150

    % Target Conditions
    theta_target = 0; % [rad]
    dockLength = 0.4; % [m]
    rotRate_target = 1; % [rpm]
    angVel_target = rotRate_target*2*pi/60; % [rad/s]
    r_target = dockLength*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m]
    v_target = angVel_target*norm(r_target)*[cos(angVel_target*tof +pi);sin(angVel_target*tof +pi)]; % [m/s]
    
    % Waypoint Conditions
%     r_waypoint = r_target;
%     v_waypoint = v_target;
%     r_waypoint = 1.3*dockLength*[cos(angVel_target*tof);sin(angVel_target*tof)]; % [m]
%     v_waypoint = angVel_target*norm(r_waypoint)*[cos(angVel_target*tof +pi/2);sin(angVel_target*tof +pi/2)]; % [m/s]
r_waypoint = [-0.137427067615333;-0.538714572652621;]
v_waypoint = [-0.005;0.002];
    
    tof_w = 0.9*tof;
    

    [r_chaser_leg1_ZEM,r_chaser_leg2_ZEM,a_ZEM] = RotDockingCostZEM(r_waypoint,v_waypoint,tof_w,tof);
    
    
end

load('OLSoln2.mat');
OLPos = [state(:,1)'; state(:,2)'];
OLVel = [state(:,3)'; state(:,4)'];
d_time = numel(state(:,1))/150;

for i = 1:numel(state(:,3))-1
OL_accel(i) = (norm(OLVel(:,i+1)-OLVel(:,i)));
end
OL_accel = sum(OL_accel);

%% Ploting and Legend Placing
    plot(OLPos(1,:),OLPos(2,:),'color',[0.9412 0.4706 0])
load('OLSolnKap.mat');
OLPos = [state(:,1)'; state(:,2)'];
OLVel = [state(:,3)'; state(:,4)'];
d_time = numel(state(:,1))/150;
plot(OLPos(1,:),OLPos(2,:),'color',[0.9412 0.4706 0])    
%     plot(r_chaser_leg1_LQR(1,:),r_chaser_leg1_LQR(2,:),'color','blue')
%     plot(r_chaser_leg1_MIX(1,:),r_chaser_leg1_MIX(2,:),'color','green')
%     plot(r_chaser_leg1_ZEM(1,:),r_chaser_leg1_ZEM(2,:),'color','red')
%     plot(r_chaser_leg2_LQR(1,:),r_chaser_leg2_LQR(2,:),'color','blue')
%     plot(r_chaser_leg2_MIX(1,:),r_chaser_leg2_MIX(2,:),'color','green')
%     plot(r_chaser_leg2_ZEM(1,:),r_chaser_leg2_ZEM(2,:),'color','red')
    legend('OL Solution','LQR','Mixed','Pure ZEM/ZEV')
    h = plot(xunit, yunit,'color','black');
    grid on
    plot([0;r_target(1,1)],[0;r_target(2,1)],'color','black')