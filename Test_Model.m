clc;
clear all;


delta_d = 0:0.05:10;
x0 = 0.5;
k = -log(0.5)/x0;

f = exp(-delta_d*k);
plot(delta_d, f, 'blue');