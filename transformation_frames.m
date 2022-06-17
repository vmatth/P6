%% Setup
clear
clc;
close all;
%These values are given from the hand eye calibration
x = 0.0858675
y = -0.480909
z = 1.13782
roll = 3.13499
pitch = 0.105303
yaw = -0.0917309
%% Convert to rpy from rad to deg
roll = rad2deg(roll)
pitch = rad2deg(pitch)
yaw = rad2deg(yaw)
%% Create rotation matrices
Rx = rotx(roll)
Ry = roty(pitch)
Rz = rotz(yaw)
R = Rz*Ry*Rx % Fixed angles
%Animate
figure()
tranimate(Rx)
tranimate(Rx,R) %movement Rx->R1

%Plot
figure()
H=trplot()
hold on
trplot(Rx,'color', 'r')
hold on
trplot(R,'color', 'g')
title('Fixed angles')

%% The transformation frame
T = [0.5010    0.7962   -0.3392 0.0858675;
    0.8294   -0.3298    0.4509 -0.480904;
    0.2472   -0.5072   -0.8256 1.13782;
    0 0 0 1]
%% Testing with points
% p = x,y,z,1
p = [0, 0, 1.1, 1]'

result = T*p





