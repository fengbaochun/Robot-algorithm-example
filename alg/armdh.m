clear;
clc;
%建立机器人模型
%       theta    d        a        alpha     offset
L1=Link([0       0.4      0.025    pi/2      0     ]);  %定义连杆的D-H参数
L2=Link([pi/2    0        0.56     0         0     ]);
L3=Link([0       0        0.035    pi/2      0     ]);
L4=Link([0       0.515    0        pi/2      0     ]);
L5=Link([pi      0        0        pi/2      0     ]);
L6=Link([0       0.08     0        0         0     ]);
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','f');       %连接连杆，f

theta = [0,0,0,0,0,0]*pi/180;                           %初始位姿的关节角度
startT = robot.fkine(theta);                            %fkine正解函数，根据我们给定的关节角theta，求解出末端位姿p

startT = double(startT)*double(transl(-0.50, 0, 0.85)); %定义初始姿态
q=robot.ikine(startT);
robot.plot(q);

% 插补参数
startPoint = [0,0,0];       %插补起点
endPoint = [0.4,0.4,0.4];   %插补终点
speed = 0.050;              %插补速度
t0 = startT;                %初始位姿
[x,y,z,T] = lineInterpolation(t0,startPoint,endPoint,speed);    %插补结果
disp(T)
% plot3(x,y,z);%输出末端轨迹
plot3(x,y,z,'-o','Color','b','MarkerSize',3,'MarkerFaceColor','#D9FFFF')

%循环显示
for i=1:1:50
    q=robot.ikine(T);           %逆解输出位姿p，求解出关节角q
    robot.plot(q);              %机械臂显示    
end


