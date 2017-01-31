clc;
clear;
close all;

% Import Environment
%==========================================================================
% Environment Variables
len = 7;   % environment length (m)
wid = 7;   % environment width (m)
% End Import Environment
%--------------------------------------------------------------------------

% Generate Nodes
%==========================================================================
% Node Params
nodeRes = 8;
node = generateNode([len/2,wid/2],nodeRes, 0, 'Hallway');
% End Generate Nodes
%--------------------------------------------------------------------------

% Simulation
%--------------------------------------------------------------------------
kk = 1;
for ii = 1:0.1:wid
    hh = 1;
    for jj = 1:0.1:len
        rssi(kk,hh) = getRSSI(node,[ii,jj]);
        hh = hh + 1;
    end
    kk = kk + 1;
end
% End Simulation
%--------------------------------------------------------------------------
figure();
surf(rssi);
title('Hallway Node Radiaton Pattern');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('RSSI (-dBm)');
set(gca,'XTickLabel',1:1:wid+1 );
set(gca,'YTickLabel',1:1:len+1 );