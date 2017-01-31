clc;
clear all;
close all;

node = generateNode([0,5],1,0,'Classroom')
plot(node.dist,node.rays,'blue'),hold on,plot(node.dist,node.st,'green');
xlabel('Distance in m');
ylabel('RSSI in -dBm');
title('RSSI vs. Distance - Simulated');
ylim([20 70]);
figure();
% Open File
data = xlsread('EPIC Classroom 1249 v2 Vertical.xlsx');
distances = data(2:41,1);
measurements = data(2:41,2:end);
[st, n, A] = fitLogDistanceModel(measurements(:,1),distances)
plot(distances,measurements(:,1),'blue'),hold on,plot(distances,st,'green');
xlabel('Distance in m');
ylabel('RSSI in -dBm');
title('RSSI vs. Distance - Measured');
ylim([20 70]);