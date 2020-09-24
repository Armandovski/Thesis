% Author: Armando Alvarez Rolins
% Super dank paper with Stephen Kwok Choon for AIAA
% June 24th, 2018
% Title: Experimental validation of ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method

function f = EXP_ZEM_ZEV_prop_waypoint(t, y)
 r=[y(1),y(2)];
 global t_go1 r_waypoint v_waypoint tof_waypoint;
    t_go1 = tof_waypoint - t;
    if t_go1 < 1e-02
        t_go1 = 1e-02;
    end
    
    r_p = [y(1); y(2)] + [y(3); y(4)].*t_go1;
    v_p = [y(3); y(4)];
    zem = r_waypoint-r_p;
    zev = v_waypoint-v_p;
    
    u(1) = (6/t_go1^2)*zem(1)-(2/t_go1)*zev(1);
    u(2) = (6/t_go1^2)*zem(2)-(2/t_go1)*zev(2);
    
    f = zeros (6,1);
    f(1) = y(3);
    f(2) = y(4);
    f(3) = (6/t_go1^2)*zem(1)-(2/t_go1)*zev(1)+3*0.0011^2*y(1)+2*0.0011*y(4);
    f(4) = (6/t_go1^2)*zem(2)-(2/t_go1)*zev(2)-2*0.0011*y(3);
    f(5) = u(1);
    f(6) = u(2);
    
end