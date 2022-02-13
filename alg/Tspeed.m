% %% 主函数 名字必须与文件一致
% function Tspeed()
%     %% input para 
%     %% q0和v0分别是起始点的位置和速度, 
%     %% q1和v1分别是终点的位置和速度。
%     %% vmax, amax分别是是规划的最大速度和最大加速度。
%     clear;
%     q0 = 100;     v0 = 50; 
%     q1 = 300;   v1 =50;
%     vmax = 150; 
%     amax = 100;
%     dmax=-200;
% 
%     [time, q, qd, qdd] = TSpeedCruveMethod(q0,q1,v0,v1,vmax,amax,dmax);
% 
%     figure(1)
%     subplot(311)
%     plot(time,q,'r','LineWidth',1.5);
%     grid on;xlabel('time[s]');ylabel('position[mm]');
% 
%     subplot(312)
%     plot(time,qd,'b','LineWidth',1.5);
%     grid on;xlabel('time[s]');ylabel('speed[mm/s]');
% 
%     subplot(313)
%     plot(time,qdd,'g','LineWidth',1.5);
%     grid on;xlabel('time[s]');ylabel('acceleration[mm/s2]');
% end

% function [time, q, qd, qdd] = TSpeedCruveMethod(q0,q1,v0,v1,vmax,amax,dmax)

function [time, q, qd, qdd] = Tspeed(q0,q1,v0,v1,vmax,amax,dmax)
    %% input q0,q1,vo,v1,vmax,amax
    %% output time,q,qd,qdd
    h = q1-q0;
    % 可达到的最大速度
    v_temp = sqrt((2.0*amax*dmax*h - amax*v1^2 + dmax*v0^2) / (dmax-amax));

    % 确定匀速阶段速度
    if(v_temp<vmax)
        vlim = v_temp;
    else
        vlim = vmax;
    end

    % 计算加速阶段的时间和位移
    Ta = (vlim-v0)/amax;
    Sa = v0*Ta+(1.0/2.0)*amax*Ta^2;

    % 计算匀速阶段的时间和位移
    Tv = (h-(vlim^2-v0^2)/(2*amax)-(v1^2-vlim^2)/(2*dmax))/vlim;
    Sv = vlim*Tv;

    % 计算减速阶段的时间和位移
    Td = (v1-vlim)/dmax;
    Sd = vlim*Td + (1.0/2.0)*dmax*Td^2;

    T = Ta + Tv +Td;
    disp("----------------------------------------------------")
    disp("q0 = " + q0 + ", v0 = " + v0)
    disp("q1 = " + q1 + ", v1 = " + v1)
    disp("vmax = " + vmax + ", vlim = " + vlim + ", amax = " + amax)
    disp("ta = " + Ta + ", sa = " + Sa)
    disp("tv = " + Tv + ", sv = " + Sv)
    disp("td = " + Td + ", sd = " + Sd)
    disp("t = " + T)
    disp("----------------------------------------------------")

    td = 0.010;
    k = 1;
    % 计算轨迹的离散点
    for t = 0:td:T
        time(k) = td *k;
        if(t >= 0 && t < Ta)
            q(k) = q0 + v0*t + (1.0/2.0)*amax*t^2;
            qd(k) = v0 + amax*t;
            qdd(k) = amax;
        elseif(t >= Ta && t < Ta+Tv)
            q(k) = q0 + Sa + vlim*(t - Ta);
            qd(k) = vlim;
            qdd(k) = 0;
        elseif(t >= Ta+Tv && t <= T)
            q(k) = q0 + Sa + Sv + vlim*(t - Ta - Tv) + (1.0/2.0)*dmax*power(t - Ta - Tv, 2);
            qd(k) = vlim + dmax *(t - Ta - Tv);
            qdd(k) = dmax;
        end
        disp("k = " + k + ", t = " + time(k)+ ", q = " + q(k)+ ", qd = " + qd(k)+", qdd = " + qdd(k))
        k = k + 1;
    end

end