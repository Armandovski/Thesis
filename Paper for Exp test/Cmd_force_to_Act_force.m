% Author: Armando Alvarez Rolins
% Super dank paper with Stephen Kwok Choon for AIAA
% June 24th, 2018
% Title: Experimental validation of ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method

function f = Cmd_force_to_Act_force(F_cmd, F_tot)
F_x = F_cmd(1);
F_x_act = F_x/F_tot;
F_y = F_cmd(2);
F_y_act = F_y/F_tot;

if (F_x_act < 0.5 && F_x_act > 0) % 0 < F_x < 0.25
    Sol_1256 = [0; 0; 0; 0;];
elseif (F_x_act < 0.75 && F_x_act > 0.25) % 0.25 < F_x < 0.75
    Sol_1256 = [1; 0; 0; 0;];
elseif F_x_act >= 0.75 % F_x >= 0.75
    Sol_1256 = [1; 1; 0; 0;];
elseif (F_x_act < 0 && F_x_act > -0.25) % -0.25 < F_x < 0
    Sol_1256 = [0; 0; 0; 0;];
elseif (F_x_act > -0.75 && F_x_act < -0.25) % -0.75 < F_x < -0.25
    Sol_1256 = [0; 0; 1; 0;];
elseif F_x_act >= 0.75 % F_x <= -0.75
    Sol_1256 = [0; 0; 1; 1;];
else
    Sol_1256 = [0; 0; 0; 0;];
end

if (F_y_act < 0.5 && F_y_act > 0) % 0 < F_y < 0.25
    Sol_3478 = [0; 0; 0; 0;];
elseif (F_y_act < 0.75 && F_y_act > 0.25) % 0.25 < F_y < 0.75
    Sol_3478 = [1; 0; 0; 0;];
elseif F_y_act >= 0.75 % F_y >= 0.75
    Sol_3478 = [1; 1; 0; 0;];
elseif (F_y_act < 0 && F_y_act > -0.25) % -0.25 < F_y < 0
    Sol_3478 = [0; 0; 0; 0;];
elseif (F_y_act > -0.75 && F_y_act < -0.25) % -0.75 < F_y < -0.25
    Sol_3478 = [0; 0; 1; 0;];
elseif F_y_act >= 0.75 % F_y <= -0.75
    Sol_3478 = [0; 0; 1; 1;];
else
    Sol_3478 = [0; 0; 0; 0;];
end

Sol_fire = [Sol_1256(1); Sol_1256(2); Sol_3478(1); Sol_3478(2); 
    Sol_1256(3); Sol_1256(4); Sol_3478(3); Sol_3478(4)];

f = Sol_fire;

end