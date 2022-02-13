
function ddh = demo()
clear ; clc; close all;
% theta1 = 0;
% theta2 = 0;
% theta3 = 0;

% syms theta d a alph

% DH 参数
a = [0,-0.42500,-0.39225,0,0,0];
d = [0.089159,0,0,0.10915,0.09465,0.08230];
alph = [pi/2,0,0,pi/2,-pi/2,0];
theta = [1,1,1,1,1,1];

T01 = transformJoint(theta(1), d(1), a(1), alph(1))
T12 = transformJoint(theta(2), d(2), a(2), alph(2))
T23 = transformJoint(theta(3), d(3), a(3), alph(3))
T34 = transformJoint(theta(4), d(4), a(4), alph(4))
T45 = transformJoint(theta(5), d(5), a(5), alph(5))
T56 = transformJoint(theta(6), d(6), a(6), alph(6))

disp("--------------------------------------------")
disp(T01*T12*T23*T34*T45*T56)
disp("--------------------------------------------")

% syms theta1 theta2 theta3
% para = [
%     0, 0, 0, theta1;
%     10, 0, 0, theta2;
%     20, -90/180*pi, 0, theta3;
%     0, 0, 30, 0];
% 
% T = DH(para)
% disp(simplify(T))

end

function T = transformJoint(theta, d, a, alph)
    % 标准 DH 正解公式
    T = rotZ(theta)*transZ(d)*transX(a)*rotX(alph) %通过两步旋转两步平移 完成相邻关节之间的变换
end


function rot_X_matrix = rotX(theta)
    % 计算绕X轴旋转的矩阵
    rot_X_matrix = [
        1, 0, 0, 0;
        0, cos(theta), -sin(theta), 0;
        0, sin(theta), cos(theta), 0;
        0, 0, 0, 1];
end

function rot_Y_matrix = rotY(theta)
    % 计算绕Y轴旋转的矩阵
    rot_Y_matrix = [
       cos(theta), 0, sin(theta), 0;
       0, 1, 0, 0;
       -sin(theta), 0, cos(theta), 0;
       0, 0, 0, 1];
end

function rot_Z_matrix = rotZ(theta)
    % 计算绕Z轴旋转的矩阵
    rot_Z_matrix = [
       cos(theta), -sin(theta), 0, 0;
       sin(theta), cos(theta), 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];
end

function trans_X_matrix = transX(scaler)
    % 计算沿X轴平移的向量
    trans_X_matrix = [
        1, 0, 0, scaler;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
end

function trans_Y_matrix = transY(scaler)
    % 计算沿Y轴平移的向量
    trans_Y_matrix = [
       1, 0, 0, 0;
       0, 1, 0, scaler;
       0, 0, 1, 0;
       0, 0, 0, 1];
end

function trans_Z_matrix = transZ(scaler)
    % 计算沿Z轴平移的向量
    trans_Z_matrix = [
       1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, scaler;
       0, 0, 0, 1];
end

function Matrix_DH = DH(DH_parameter)
    % 输入参数为DH参数表，参数表有四列，参数表的行数是运动轴的个数。比如六自由度机械臂的参数表为6行4列
    % 第一列是Z轴偏置trans_X；
    % 第二列是Z轴偏转角theta_X；
    % 第三列是X轴偏置trans_Z；
    % 第四列是X轴偏转角theta_Z；

    [dim_row, dim_col] = size(DH_parameter);
    Matrix_DH = eye(4);

    for row_index = 1:dim_row
        % 先绕X轴转theta_X(Z轴偏转角)， 再沿X轴平移trans_X(Z轴偏置)，将两个坐标系的Z轴重合
        theta_X = DH_parameter(row_index, 2);
        trans_X = DH_parameter(row_index, 1);
        T_X = rotX(theta_X) * transX(trans_X);

        % 先绕Z轴转theta_Z(X轴偏转角)， 再沿Z轴平移trans_Z(X轴偏置)，将两个坐标系的X轴重合
        theta_Z = DH_parameter(row_index, 4);
        trans_Z = DH_parameter(row_index, 3);
        T_Z = rotZ(theta_Z) * transZ(trans_Z);

        Matrix_DH = Matrix_DH * T_X * T_Z;
    end
end






