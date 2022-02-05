function main()
    clear;
    clc;
    close all;

    [robot,startT] = creatRobot();      %创建机器人
    q=robot.ikine(startT);
    robot.plot(q);                      %显示机器人
	% 插补参数
    startPoint = [-220,0,-140];  %插补起点
    endPoint    = [220,-20,140];       %插补终点
    speed = 40;                     %插补速度
    t0 = startT;                    %初始位姿
    [x,y,z,T] = lineInterpolation(t0,startPoint,endPoint,speed);    %插补结果
    % 显示轨迹
    Tj=transl(T);                   %将平移部分提取出来
    plot3(Tj(:,1),Tj(:,2),Tj(:,3),'-o','Color','b','MarkerSize',3,'MarkerFaceColor','#D9FFFF');%输出末端轨迹
    
    %循环显示
    for i=1:1:50
        q=robot.ikine(T);           %逆解输出位姿p，求解出关节角q
        robot.plot(q);              %机械臂显示    
    end

end



