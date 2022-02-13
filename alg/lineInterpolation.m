%%直线插补
function [x,y,z,T] = lineInterpolation(t0,startPoint,endPoint,speed)
    p0=startPoint;
    pf=endPoint;                %指定起止位置
    v=speed;                    %指定速度
    x=[p0(1)];y=[p0(2)];z=[p0(3)];
    L=((pf(1)-p0(1))^2+(pf(2)-p0(2))^2+(pf(3)-p0(3))^2)^0.5;%直线长度
    N=L/v;                      %插补次数
    dx=(pf(1)-p0(1))/N;         %每个周期各轴增量
    dy=(pf(2)-p0(2))/N;
    dz=(pf(3)-p0(3))/N;

    for t=1:1:N                 %在初始位姿上进行 插补增量计算
        x(t+1)=x(t)+dx;
        y(t+1)=y(t)+dy;
        z(t+1)=z(t)+dz;
        T(:,:,t)=double(t0)*double(transl(x(t+1), y(t+1), z(t+1)));    %在基础位姿上插补
    end
    x = x + t0(1,4);            %需考虑初始位置
    y = y + t0(2,4);
    z = z + t0(3,4);
%     plot3(x,y,z,'r'),xlabel('x'),ylabel('y'),zlabel('z'),hold on,plot3(x,y,z,'o','color','g'),grid on;
end



