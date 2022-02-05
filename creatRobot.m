function [robot,startT]= creatRobot()
    % 标准DH
    % DH参数   1           2           3           4           5            6
    a       = [0,         -425,        -392.25,    0,          0,           0         ];
    d       = [89.459,     0,          0,          109.15,     94.65,       82.30     ];
    alph    = [pi/2,       0,          0,          pi/2,       -pi/2,       0         ];% alpha没有用到,故此逆解程序只适合alpha=[pi/2,0,0,pi/2,-pi/2,0]的情况！

    %        theta   d            a        alpha      sigma
    L1=Link([  0,    d(1),       a(1),    alph(1),      0 ],'standard');
    L2=Link([  0,    d(2),       a(2),    alph(2),      0 ],'standard');
    L3=Link([  0,    d(3),       a(3),    alph(3),      0 ],'standard');
    L4=Link([  0,    d(4),       a(4),    alph(4),      0 ],'standard');
    L5=Link([  0,    d(5),       a(5),    alph(5),      0 ],'standard');
    L6=Link([  0,    d(6),       a(6),    alph(6),      0 ],'standard');
    robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','standard DH');
    robot.name = 'UR5';
    robot.display();                    %输出机器人信息

    theta = [0,0,0,0,0,0]*pi/180;       %初始位姿的关节角度
    startT = robot.fkine(theta);   

    startT = double(startT)*double(transl(400, 200, 300))*double(rotY(-pi/4))*double(rotX(pi/2)); %定义初始姿态
end

function rot_Y_matrix = rotY(theta)
    % 计算绕Y轴旋转的矩阵
    theta = sym(theta);
    rot_Y_matrix = [
       cos(theta), 0, sin(theta), 0;
       0, 1, 0, 0;
       -sin(theta), 0, cos(theta), 0;
       0, 0, 0, 1];
%     disp(rot_Y_matrix)
end

function rot_X_matrix = rotX(theta)
    % 计算绕X轴旋转的矩阵
    theta = sym(theta);
    rot_X_matrix = [
        1, 0, 0, 0;
        0, cos(theta), -sin(theta), 0;
        0, sin(theta), cos(theta), 0;
        0, 0, 0, 1];
%     disp(rot_X_matrix)
end



