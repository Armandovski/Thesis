% read C output files
clc;
clear all;
close all;

filename = 'poscost5.txt';
Ain = load(filename);
nn = length(Ain);
n1 = Ain(nn,1)+1; n2 = Ain(nn,2)+1;
M = zeros(n1,n2); minV = cell(n1,n2);
minVx = M; minVy = M;
for i = 1:n1;
    for j = 1:n2
        index = (i-1)*n2 + j;
        M(i,j) = Ain(index,3);
        minVx(i,j) = Ain(index,4);
        minVy(i,j) = Ain(index,5);
        if (M(i,j) > 10)
            minVx(i,j) = 0;
            minVy(i,j) = 0;
        end
       % minV{i,j} = [Ain(index,4); Ain(index,5)];
    end
end
x = linspace(-2.5,2.5,n1);
y = linspace(-1.3,1.3,n2);

figure(1)
contourf(x,y,min(log10(M'),1))