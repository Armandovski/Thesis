function f = ZEM_ZEV_prop(t, y)
 r=[y(1),y(2)];
 global tof r_target v_target t_go2;
    t_go2 = tof - t;
     if t_go2 < 1e-02
        t_go2 = 1e-02;
    end
    
    r_p = [y(1); y(2)] + [y(3); y(4)].*t_go2;
    v_p = [y(3); y(4)];
    zem = r_target-r_p;
    zev = v_target-v_p;
    
    f = zeros (4,1);
    f(1) = y(3);
    f(2) = y(4);
    f(3) = (6/t_go2^2)*zem(1)-(2/t_go2)*zev(1)+3*0.0011^2*y(1)+2*0.0011*y(4);
    f(4) = (6/t_go2^2)*zem(2)-(2/t_go2)*zev(2)-2*0.0011*y(3);
    f(5) = norm((6/t_go2^2)*zem(1)-(2/t_go2)*zev(1));
    f(6) = norm((6/t_go2^2)*zem(2)-(2/t_go2)*zev(2));
    
end