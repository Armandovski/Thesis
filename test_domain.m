% Author: Armando Alvarez Rolins
% Master's Thesis
% Aug. 29th, 2016
% Title: ZEM/ZEV Rendezvous on Rotating Target with Waypoint Method
% File: Test Plot data

n1 = 100; n2 = 50; n3 = 100;
%warning('off','all')
x = linspace(-2.5, 2.5, n1);
y = linspace(-1.3, 1.3, n2);
Amin = zeros(n1,n2);
tof_w = 0.9*150;
tic
for i = 1:n1
    for j = 1:n2
        r_w = [x(i); y(j)];
        A = zeros(1,n3);
        for k = 1:n3
            vmag = sqrt(rand)*0.05;
            th = rand*2*pi;
            v_w(:,k) = vmag.*[cos(th); sin(th)];
            if(norm(r_w)>0.4)
                A(k) = RotDockingCost(r_w,v_w(:,k),tof_w);
            else
                A(k) = RotDockingCost(r_w,v_w(:,k),tof_w)+1;
            end
        end
        [Amin(i,j),k] = min(A);
        Vmin(i,j,:) = min(v_w(k));
         disp(num2str(j))
    end

end

figure(1)
contourf(x,y,Amin','LineStyle','none')

figure(2)
contourf(x,y,log10(Amin'),'LineStyle','none')

figure(3)
contourf(x,y,min(Amin',1e-03),'LineStyle','none')
toc
