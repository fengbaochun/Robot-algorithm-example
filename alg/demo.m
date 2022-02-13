%% 用于测试 单个算法
function demo()
    clear;
    clc;
    close all;
    TspeedDemo();                   %梯形速度规划
%     lineInterpolationDemo();        %基于初始位置的线性插补
end


%% Tspeed demo test
function TspeedDemo()
    % 参数
	q0 = 100;     v0 = 50; 
    q1 = 300;   v1 =50;
    vmax = 150; 
    amax = 100;
    dmax=-200;
	    
    % 速度规划函数调用
    [time, q, qd, qdd] = Tspeed(q0,q1,v0,v1,vmax,amax,dmax);
    % 画图
    figure(1)
    subplot(311)
    plot(time,q,'r','LineWidth',1.5);
    grid on;xlabel('time[s]');ylabel('position[mm]');

    subplot(312)
    plot(time,qd,'b','LineWidth',1.5);
    grid on;xlabel('time[s]');ylabel('speed[mm/s]');

    subplot(313)
    plot(time,qdd,'g','LineWidth',1.5);
    grid on;xlabel('time[s]');ylabel('acceleration[mm/s2]');
    
end

%% lineInterpolation demo test
function lineInterpolationDemo()
    startPoint = [-220,0,-140];         %插补起点
    endPoint    = [220,-20,140];        %插补终点
    speed = 80;                         %插补速度
    t0 = eye(4);                        %单位矩阵    
    [x,y,z,T] = lineInterpolation(t0,startPoint,endPoint,speed);    %插补结果
    disp(length(x));                    %插补点个数                     
    plot3(x,y,z,'r'),xlabel('x'),ylabel('y'),zlabel('z'),hold on,plot3(x,y,z,'o','color','g');
    grid on;
end
