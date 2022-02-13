function kinematics()
    fk()
%     ik()
end


% 机械臂正解
function T = fk()

% DH 参数  关节 1         2            3            4           5           6
    a       = [0,         -425,        -392.25,    0,          0,           0         ];
    d       = [89.459,     0,          0,          109.15,     94.65,       82.30     ];
    alph    = [pi/2,       0,          0,          pi/2,       -pi/2,       0         ];% alpha没有用到,故此逆解程序只适合alpha=[pi/2,0,0,pi/2,-pi/2,0]的情况！ 
    q       = [0.15,      1,           0.35,        1,          0.25,       1.1       ];
%     q       = [0,      0,           0,        0,          0,       0       ];
    
%     % 代数运算
%     syms q1 q2 q3 q4 q5 q6;
%     syms a1 a2 a3 a4 a5 a6;
%     syms d1 d2 d3 d4 d5 d6;
%     q = [q1,q2,q3,q4,q5,q6];
%     a = [a1,a2,a3,a4,a5,a6];
%     d = [d1,d2,d3,d4,d5,d6];
%     alph = [pi/2,0,0,pi/2,-pi/2,0];
    
    T01 = transformJoint(q(1), d(1), a(1), alph(1));
    T12 = transformJoint(q(2), d(2), a(2), alph(2));
    T23 = transformJoint(q(3), d(3), a(3), alph(3));
    T34 = transformJoint(q(4), d(4), a(4), alph(4));
    T45 = transformJoint(q(5), d(5), a(5), alph(5));
    T56 = transformJoint(q(6), d(6), a(6), alph(6));

    T = T01*T12*T23*T34*T45*T56;
%     T = simplify(T);
%     ik(T)
    niyundongxue(T);

end

% 机械臂逆解
function theta = ik(fk_T)
    T = fk_T;
    nx = T(1,1); ox = T(1,2); ax = T(1,3); x = T(1,4);
    ny = T(2,1); oy = T(2,2); ay = T(2,3); y = T(2,4);
    nz = T(3,1); oz = T(3,2); az = T(3,3); z = T(3,4);
    
    disp(nx);disp(ox);disp(ox);disp(x);
    disp(ny);disp(oy);disp(oy);disp(y);
    disp(nz);disp(oz);disp(oz);disp(z);
end

% 解析法
function theta=niyundongxue(T)
    %变换矩阵T已知
    %SDH:标准DH参数表求逆解（解析解）
    %部分DH参数表如下，需要求解theta信息
    
% DH 参数  关节 1         2            3          4           5           6
    a       = [0,         -425,        -392.25,    0,          0,           0         ];
    d       = [89.459,     0,          0,          109.15,     94.65,       82.30     ];
    alph    = [pi/2,       0,          0,          pi/2,       -pi/2,       0         ];% alpha没有用到,故此逆解程序只适合alpha=[pi/2,0,0,pi/2,-pi/2,0]的情况！
         
    nx=T(1,1);ny=T(2,1);nz=T(3,1);
    ox=T(1,2);oy=T(2,2);oz=T(3,2);
    ax=T(1,3);ay=T(2,3);az=T(3,3);
    px=T(1,4);py=T(2,4);pz=T(3,4);
    disp(double(T))
    %求解关节角1
    m=d(6)*ay-py;  n=ax*d(6)-px; 
    theta1(1,1)=atan2(m,n)-atan2(d(4),sqrt(m^2+n^2-(d(4))^2));
    theta1(1,2)=atan2(m,n)-atan2(d(4),-sqrt(m^2+n^2-(d(4))^2));
  
    %求解关节角5
    theta5(1,1:2)=acos(ax*sin(theta1)-ay*cos(theta1));
    theta5(2,1:2)=-acos(ax*sin(theta1)-ay*cos(theta1));      
    
    %求解关节角6
    mm=nx*sin(theta1)-ny*cos(theta1); nn=ox*sin(theta1)-oy*cos(theta1);
    %theta6=atan2(mm,nn)-atan2(sin(theta5),0);
    theta6(1,1:2)=atan2(mm,nn)-atan2(sin(theta5(1,1:2)),0);
    theta6(2,1:2)=atan2(mm,nn)-atan2(sin(theta5(2,1:2)),0);
    
    %求解关节角3
    mmm(1,1:2)=d(5)*(sin(theta6(1,1:2)).*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6(1,1:2)).*(ox*cos(theta1)+oy*sin(theta1))) ...
        -d(6)*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1);
    nnn(1,1:2)=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta6(1,1:2))+nz*sin(theta6(1,1:2)));
    mmm(2,1:2)=d(5)*(sin(theta6(2,1:2)).*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6(2,1:2)).*(ox*cos(theta1)+oy*sin(theta1))) ...
        -d(6)*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1);
    nnn(2,1:2)=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta6(2,1:2))+nz*sin(theta6(2,1:2)));
    theta3(1:2,:)=acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
    theta3(3:4,:)=-acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
    
    %求解关节角2
    mmm_s2(1:2,:)=mmm;
    mmm_s2(3:4,:)=mmm;
    nnn_s2(1:2,:)=nnn;
    nnn_s2(3:4,:)=nnn;
    s2=((a(3)*cos(theta3)+a(2)).*nnn_s2-a(3)*sin(theta3).*mmm_s2)./ ...
        ((a(2))^2+(a(3))^2+2*a(2)*a(3)*cos(theta3));
    c2=(mmm_s2+a(3)*sin(theta3).*s2)./(a(3)*cos(theta3)+a(2));
    theta2=atan2(s2,c2);   
    
    %整理关节角1 5 6 3 2
    theta(1:4,1)=theta1(1,1);theta(5:8,1)=theta1(1,2);
    theta(:,2)=[theta2(1,1),theta2(3,1),theta2(2,1),theta2(4,1),theta2(1,2),theta2(3,2),theta2(2,2),theta2(4,2)]';
    theta(:,3)=[theta3(1,1),theta3(3,1),theta3(2,1),theta3(4,1),theta3(1,2),theta3(3,2),theta3(2,2),theta3(4,2)]';
    theta(1:2,5)=theta5(1,1);theta(3:4,5)=theta5(2,1);
    theta(5:6,5)=theta5(1,2);theta(7:8,5)=theta5(2,2);
    theta(1:2,6)=theta6(1,1);theta(3:4,6)=theta6(2,1);
    theta(5:6,6)=theta6(1,2);theta(7:8,6)=theta6(2,2); 
    
    %求解关节角4
    theta(:,4)=atan2(-sin(theta(:,6)).*(nx*cos(theta(:,1))+ny*sin(theta(:,1)))-cos(theta(:,6)).* ...
        (ox*cos(theta(:,1))+oy*sin(theta(:,1))),oz*cos(theta(:,6))+nz*sin(theta(:,6)))-theta(:,2)-theta(:,3);  
    
%     ret = double(theta(1,1));
    disp(double(theta))
end

% 标准 DH 正解公式
function T = transformJoint(theta, d, a, alph)
    T = rotZ(theta)*transZ(d)*transX(a)*rotX(alph) %通过两步旋转两步平移 完成相邻关节之间的变换
end

function sin_=sin_(a)
	sin_=sin(a/180*pi);
end
 
function cos_=cos_(a)
    cos_=cos(a/180*pi);
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

function rot_Z_matrix = rotZ(theta)
    % 计算绕Z轴旋转的矩阵
    theta = sym(theta);
    rot_Z_matrix = [
       cos(theta), -sin(theta), 0, 0;
       sin(theta), cos(theta), 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];
%     disp(rot_Z_matrix)
end

function trans_X_matrix = transX(scaler)
    % 计算沿X轴平移的向量
    scaler = sym(scaler);
    trans_X_matrix = [
        1, 0, 0, scaler;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
%     disp(trans_X_matrix)
end

function trans_Y_matrix = transY(scaler)
    % 计算沿Y轴平移的向量
    scaler = sym(scaler);
    trans_Y_matrix = [
       1, 0, 0, 0;
       0, 1, 0, scaler;
       0, 0, 1, 0;
       0, 0, 0, 1];
% 	disp(trans_Y_matrix)
end

function trans_Z_matrix = transZ(scaler)
    % 计算沿Z轴平移的向量
    scaler = sym(scaler);
    trans_Z_matrix = [
       1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, scaler;
       0, 0, 0, 1];
% 	disp(trans_Z_matrix)   
end







