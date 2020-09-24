% Author: Armando Alvarez Rolins
% Super dank paper with Stephen Kwok Choon for AIAA
% June 24th, 2018
% Title: Experimental validation of ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method

function f = Cmd_torque_to_Act_torque(T_cmd, T_tot)
T_act = T_cmd/T_tot;

if T_act > 0
    if (T_act <=0 && T_act < 0.125)
        Sol_1357 = [0; 0; 0; 0;];
    elseif (T_act < 0.375 && T_act >= 0.125)
        Sol_1357 = [1; 0; 0; 0;];
    elseif (T_act < 0.625 && T_act >= 0.375)
        Sol_1357 = [1; 0; 1; 0;];
    elseif (T_act < 0.875 && T_act >= 0.625)
        Sol_1357 = [1; 1; 1; 0;];
    elseif (T_act >= 0.875)
        Sol_1357 = [1; 1; 1; 1;];
    else
        Sol_1357 = [0; 0; 0; 0;];
    end
Sol_2468 = [0; 0; 0; 0;];
end

if T_act < 0
    T_act = abs(T_act);
        if (T_act <=0 && T_act < 0.125)
            Sol_2468 = [0; 0; 0; 0;];
        elseif (T_act < 0.375 && T_act >= 0.125)
            Sol_2468 = [1; 0; 0; 0;];
        elseif (T_act < 0.625 && T_act >= 0.375)
            Sol_2468 = [1; 0; 1; 0;];
        elseif (T_act < 0.875 && T_act >= 0.625)
            Sol_2468 = [1; 1; 1; 0;];
        elseif (T_act >= 0.875)
            Sol_2468 = [1; 1; 1; 1;];
        else
            Sol_2468 = [0; 0; 0; 0;];
        end
Sol_1357 = [0; 0; 0; 0;];
end

Sol_fire = [Sol_1357(1); Sol_2468(1); Sol_1357(2); Sol_2468(2);
    Sol_1357(3); Sol_2468(3); Sol_1357(4); Sol_2468(4)];

f = Sol_fire;
end