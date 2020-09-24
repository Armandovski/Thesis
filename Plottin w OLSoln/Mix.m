% Author: Armando Alvarez Rolins
% Master's Thesis
% Aug. 29th, 2016
% Title: ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method

function f = Mix(t,y)
y_state = [y(1);y(2);y(3);y(4)];
global tof r_target v_target t_go2;
t_go2 = tof - t;
if (t_go2 < 1e-2)
    t_go2 = 1e-2;
end
n = 0.0011;

y_target = [r_target(1); r_target(2); v_target(1); v_target(2)];
A_hat = [4-3*cos(n*t_go2) 0 (1/n)*sin(n*t_go2) -(2/n)*(cos(n*t_go2)-1);
    6*sin(n*t_go2)-6*n*t_go2 1 (2/n)*(cos(n*t_go2)-1) (4/n)*sin(n*t_go2)-3*t_go2;
    3*n*sin(n*t_go2) 0 cos(n*t_go2) 2*sin(n*t_go2);
    6*n*(cos(n*t_go2)-1) 0 -2*sin(n*t_go2) 4*cos(n*t_go2)-3];
phi_rr = [A_hat(1,1) A_hat(1,2);
    A_hat(2,1) A_hat(2,2)];
phi_rv = [A_hat(1,3) A_hat(1,4);
    A_hat(2,3) A_hat(2,4)];
phi_vr = [A_hat(3,1) A_hat(3,2);
    A_hat(3,1) A_hat(3,2)];
phi_vv = [A_hat(4,3) A_hat(4,4);
    A_hat(4,3) A_hat(4,4)];

ZEM = [y_target(1);y_target(2)]-phi_rr*[y_state(1);y_state(2)]-phi_rv*[y_state(3);y_state(4)];
ZEV = [y_target(3);y_target(4)]-phi_vr*[y_state(1);y_state(2)]-phi_vv*[y_state(3);y_state(4)];


% B_hat = [   ((13*sin(n*t_go2))/2 - n*(4*t_go2 + (5*t_go2*cos(n*t_go2))/2))/n^3    -(3*n^2*t_go2^2 - 32*sin((n*t_go2)/2)^2 + 5*n*t_go2*sin(n*t_go2))/n^3 (16*sin((n*t_go2)/2)^2 - 5*n*t_go2*sin(n*t_go2))/(2*n^2)            (6*t_go2 + 5*t_go2*cos(n*t_go2))/n - (11*sin(n*t_go2))/n^2;
%             (3*n^2*t_go2^2 - 32*sin((n*t_go2)/2)^2 + 5*n*t_go2*sin(n*t_go2))/n^3 (38*sin(n*t_go2) - n*(28*t_go2 + 10*t_go2*cos(n*t_go2)))/n^3 + (3*t_go2^3)/2   (11*sin(n*t_go2))/n^2 - (6*t_go2 + 5*t_go2*cos(n*t_go2))/n (56*sin((n*t_go2)/2)^2 - 10*n*t_go2*sin(n*t_go2))/n^2 - (9*t_go2^2)/2;
%             -(16*sin((n*t_go2)/2)^2 - 5*n*t_go2*sin(n*t_go2))/(2*n^2)               (11*sin(n*t_go2))/n^2 - (6*t_go2 + 5*t_go2*cos(n*t_go2))/n        (3*sin(n*t_go2))/(2*n) - (5*t_go2*cos(n*t_go2))/2                  (12*sin((n*t_go2)/2)^2)/n - 5*t_go2*sin(n*t_go2);
%             (6*t_go2 + 5*t_go2*cos(n*t_go2))/n - (11*sin(n*t_go2))/n^2    (9*t_go2^2)/2 - (56*sin((n*t_go2)/2)^2 - 10*n*t_go2*sin(n*t_go2))/n^2         5*t_go2*sin(n*t_go2) - (12*sin((n*t_go2)/2)^2)/n                 (18*sin(n*t_go2))/n - 10*t_go2*cos(n*t_go2) - 9*t_go2]
        

u = 6*ZEM/t_go2^2 -2*ZEV/t_go2;

f = zeros(6,1);
    f(1) = y(3);
    f(2) = y(4);
    f(3) = u(1)+3*0.0011^2*y(1)+2*0.0011*y(4);
    f(4) = u(2)-2*0.0011*y(3);
    f(5) = norm(u(1));
    f(6) = norm(u(2));
end