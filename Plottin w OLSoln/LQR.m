% Author: Armando Alvarez Rolins
% Master's Thesis
% Aug. 29th, 2016
% Title: ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method

function f = LQR(t,y)
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
ZEM = y_target-A_hat*y_state;

S = (sin(n*t_go2/2)^2);

B_hat = [   ((13*sin(n*t_go2))/2 - n*(4*t_go2 + (5*t_go2*cos(n*t_go2))/2))/n^3    -(3*n^2*t_go2^2 - 32*sin((n*t_go2)/2)^2 + 5*n*t_go2*sin(n*t_go2))/n^3 (16*sin((n*t_go2)/2)^2 - 5*n*t_go2*sin(n*t_go2))/(2*n^2)            (6*t_go2 + 5*t_go2*cos(n*t_go2))/n - (11*sin(n*t_go2))/n^2;
            (3*n^2*t_go2^2 - 32*sin((n*t_go2)/2)^2 + 5*n*t_go2*sin(n*t_go2))/n^3 (38*sin(n*t_go2) - n*(28*t_go2 + 10*t_go2*cos(n*t_go2)))/n^3 + (3*t_go2^3)/2   (11*sin(n*t_go2))/n^2 - (6*t_go2 + 5*t_go2*cos(n*t_go2))/n (56*sin((n*t_go2)/2)^2 - 10*n*t_go2*sin(n*t_go2))/n^2 - (9*t_go2^2)/2;
            -(16*sin((n*t_go2)/2)^2 - 5*n*t_go2*sin(n*t_go2))/(2*n^2)               (11*sin(n*t_go2))/n^2 - (6*t_go2 + 5*t_go2*cos(n*t_go2))/n        (3*sin(n*t_go2))/(2*n) - (5*t_go2*cos(n*t_go2))/2                  (12*sin((n*t_go2)/2)^2)/n - 5*t_go2*sin(n*t_go2);
            (6*t_go2 + 5*t_go2*cos(n*t_go2))/n - (11*sin(n*t_go2))/n^2    (9*t_go2^2)/2 - (56*sin((n*t_go2)/2)^2 - 10*n*t_go2*sin(n*t_go2))/n^2         5*t_go2*sin(n*t_go2) - (12*sin((n*t_go2)/2)^2)/n                 (18*sin(n*t_go2))/n - 10*t_go2*cos(n*t_go2) - 9*t_go2]
        
% B_hat = [(13/(2*n^3))*sin(n*t_go2)-13*t_go2/(2*n^2)-5*t_go2*S/n^2 (16*S/n^3)-(3*t^2)/n 8*S/n^2-(5/(2*n))*t_go2*sin(n*t_go2) -(11/n^2)*sin(n*t_go2)-10*S*t_go2/n;
% -32*S/n^3+3*t_go2^2/n+(5/n^2)*t_go2*sin(n*t_go2) -(16/n^2)*t_go2+(3/2)*t_go2^3+(16/n^3)*sin(n*t_go2) (11/n^2)*sin(n*t_go2)-(11/n)*t_go2+(10*S/n)*t_go2 56*S/n^2-9*t_go2/2-10*t_go2*sin(n*t_go2)/n;
% 5*t_go2*sin(n*t_go2)/(2*n)-8*S/n^2 8*sin(n*t_go2)/n^2-6*t_go2/n 3*sin(n*t_go2)/(2*n)+5*t_go2*(2*S-1)/2 12*S/n-5*t_go2*sin(n*t_go2);
% 11*t_go2/n-11*sin(n*t_go2)/n^2-10*S*t_go2/n 9*t_go2^2/2-32*S/n^2 -12*S/n+5*t_go2*sin(n*t_go2) -19*t_go2+18*sin(n*t_go2)/n+20*S*t_go2];

p_0 = inv(B_hat)*ZEM;

f = zeros(6,1);
    f(1) = y(3);
    f(2) = y(4);
    f(3) = -p_0(3)+3*0.0011^2*y(1)+2*0.0011*y(4);
    f(4) = -p_0(4)-2*0.0011*y(3);
    f(5) = norm(-p_0(3));
    f(6) = norm(-p_0(4));
end