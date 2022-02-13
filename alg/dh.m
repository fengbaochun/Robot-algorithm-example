clc
clear
close all
%% 标准DH

%        theta   d              a        alpha      sigma
L1=Link([  0     89.459       0           pi/2          0   ],'standard');
L2=Link([  0     0            -425        0             0   ],'standard');
L3=Link([  0     0            -392.25     0             0   ],'standard');
L4=Link([  0     109.15       0           pi/2          0   ],'standard');
L5=Link([  0     94.65        0          -pi/2          0   ],'standard');
L6=Link([  0     82.3         0           0             0   ],'standard');
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','standard DH');

theta = [0, 0, 0, 0, 0, 0];
% theta = [90, 45, 0, 0, 0, 0]*pi/180;
% theta = [0, 90, 90, 90, 90, 90]*pi/180;
robot.plot(theta);          %输出机器人模型，theta为6个轴的角度
robot.display();            %输出机器人信息
t0 = robot.fkine(theta)     %正解，根据关节角求出末端


