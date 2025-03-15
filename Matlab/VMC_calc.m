function [Tp1,Tp2] = VMC_calc(F0,Tp,phi1,phi4)
% syms T1 T2 F0 Tp phi1 phi4 XE l1 l2 l3 l4
    l5=0.08;
    l1=0.075;
    l2=0.14;
    l3=0.14;
    l4=0.075;

% 计算坐标B,D
YD = l4*sin(phi4);
YB = l1*sin(phi1);
XD = l5 + l4*cos(phi4);
XB = l1*cos(phi1); 
lBD = sqrt((XD - XB)*(XD - XB) + (YD - YB)*(YD - YB));

% 计算phi2和phi3
A0 = 2*l2*(XD - XB);
B0 = 2*l2*(YD - YB);
C0 = l2*l2 + lBD*lBD - l3*l3;
phi2 = 2*atan2((B0 + sqrt(A0*A0 + B0*B0 - C0*C0)),A0 + C0);
phi3 = atan2(YB-YD+l2*sin(phi2),XB-XD+l2*cos(phi2));

% 计算C和腿长，角度
XC = l1*cos(phi1) + l2*cos(phi2);
YC = l1*sin(phi1) + l2*sin(phi2);
L0 = sqrt((XC - l5/2)*(XC - l5/2) + YC*YC);
phi0 = atan2(YC,XC - l5/2);

% 计算雅可比矩阵，变量之间的关系
j11 = (l1*sin(phi0-phi3)*sin(phi1-phi2))/sin(phi3-phi2);
j12 = (l1*cos(phi0-phi3)*sin(phi1-phi2))/(L0*sin(phi3-phi2));
j21 = (l4*sin(phi0-phi2)*sin(phi3-phi4))/sin(phi3-phi2);
j22 = (l4*cos(phi0-phi2)*sin(phi3-phi4))/(L0*sin(phi3-phi2));
J = [j11 j12; j21 j22];

F = [F0;Tp];% 末端执行器所需要的推力和力矩
T = J * F;
Tp1 = T(1,1);% 关节电机所需要的力矩
Tp2 = T(2,1);

% J11 = diff(L0, phi1);
% J12 = diff(L0, phi4);
% J21 = diff(phi0, phi1);
% J22 = diff(phi0, phi4);
% J = [J11, J12; J21, J22];
% T = J * F;
% Tp1 = T(1,1);
% Tp2 = T(2,1);

end