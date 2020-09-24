% read C output files
clc;
clear all;
close all;

filename = 'velcost10.txt';
Ain = load(filename);
nn = length(Ain);
n1 = Ain(nn,1)+1; n2 = Ain(nn,2)+1;
M = zeros(n1,n2); 
vx = M; vy = M;
for i = 1:n1;
    for j = 1:n2
        index = (i-1)*n2 + j;
        M(i,j) = Ain(index,5);
        vx(i,j) = Ain(index,3);
        vy(i,j) = Ain(index,4);
       % minV{i,j} = [Ain(index,4); Ain(index,5)];
    end
end
%x = linspace(-2.5,2.5,n1);
%y = linspace(-1.3,1.3,n2);

figure(1)
contourf(vx,vy,log10(M))
%contourf(vx,vy,min(log10(M),1))