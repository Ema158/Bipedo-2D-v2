clc
clear
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5prueba
d2 = 10;
d3 = 10;
q1 = deg2rad(10);
q2 = deg2rad(-30);
q3 = deg2rad(10);
q4 = deg2rad(-20);
q5 = deg2rad(30);
q6 = deg2rad(10);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Bipdeo 2D
alpha = zeros(6,1);
d = [0;d2;d3;0;0-d3;-d2];
theta = [q1;q2;q3;q4;q5;q6];
r = zeros(6,1);
T = cell(1,6);
T0 = cell(1,6);
T0M = cell(1,6);
for i=1:6
    T{i} = [cos(theta(i)) -sin(theta(i)) 0 d(i);
           cos(alpha(i))*sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i)) -r(i)*sin(alpha(i));
           sin(alpha(i))*sin(theta(i)) sin(alpha(i))*cos(theta(i)) cos(alpha(i)) r(i)*cos(alpha(i));
           0 0 0 1];
end
%%%%%%%%%%%%%%%%%%Matrices respecto a cero
T_M = [0 1 0 0;
           0 0 1 0;
           1 0 0 0;
           0 0 0 1];
T_pie = [1 0 0 0;
           0 1 0 1;
           0 0 1 0;
           0 0 0 1];
T_pie_2 = [1 0 0 0;
           0 1 0 1;
           0 0 1 0;
           0 0 0 1];
T0{1} = T{1};
T0{2} = T{1}*T{2};
T0{3} = T{1}*T{2}*T{3};
T0{4} = T{1}*T{2}*T{3}*T{4};
T0{5} = T{1}*T{2}*T{3}*T{4}*T{5};
T0{6} = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};
T0{7} = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*T_pie_2;
for i=1:7
    T0M{i} = T_M*T0{i};
    T0_pie = T_M*T_pie;
end
T0M_aux = T0M;
T0M{1} = T0_pie;
T0M{2} = T0M_aux{1};
T0M{3} = T0M_aux{2};
T0M{4} = T0M_aux{3};
T0M{5} = T0M_aux{4};
T0M{6} = T0M_aux{5};
T0M{7} = T0M_aux{6};
T0M{8} = T0M_aux{7};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Dibujo
for i=1:7
    plot([T0M{i}(1,4) T0M{i+1}(1,4)],[T0M{i}(3,4) T0M{i+1}(3,4)],'LineWidth',1.5)
    axis equal
    grid on
    hold on
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Jacobiana
