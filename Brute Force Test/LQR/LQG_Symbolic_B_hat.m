clc;
clear all;

syms p20 p30 p40 t n
n = sym('n','real');

A = [0 0 1 0;
    0 0 0 1;
    3*n^2 0 0 2*n;
    0 0 -2*n 0];

B = [0 0;
    0 0;
    1 0;
    0 1];

u = [(2/n)*p20-p30;
    -p40];

B_hat = expm(A*t)*int(-(expm(-A*t)*B*B'*expm(-A'*t)),t,0,t);

S = simple(B_hat)