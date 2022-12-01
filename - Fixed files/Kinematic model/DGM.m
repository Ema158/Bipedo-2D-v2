function T = DGM(robot)
%% computing of the geometric model (transformations matrix) of the robot
%theta = robot.q;
T=zeros(4,4,9); %%%%%%%%%%%%%10 porque agregué el marco del torso
d1 = 0.01;
d2 = 0.1;
d3 = 0.1;
q1 = robot.q(1);
q2 = robot.q(2);
q3 = robot.q(3);
q4 = robot.q(4);
q5 = robot.q(5);
q6 = robot.q(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Bipdeo 2D
alpha = zeros(6,1);
d = [0;d2;d3;0;-d3;-d2];
theta = [q1;q2;q3;q4;q5;q6];
r = zeros(6,1);
T_temp = cell(1,6);
T0 = cell(1,6);
T0M = cell(1,9);
for i=1:6
    T_temp{i} = [cos(theta(i)) -sin(theta(i)) 0 d(i);
           cos(alpha(i))*sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i)) -r(i)*sin(alpha(i));
           sin(alpha(i))*sin(theta(i)) sin(alpha(i))*cos(theta(i)) cos(alpha(i)) r(i)*cos(alpha(i));
           0 0 0 1];
end
%%%%%%%%%%%%%%%%%%Matrices respecto a cero
T_M0 =    [0 1 0 0;
           0 0 1 0;
           1 0 0 0.046;
           0 0 0 1];
T_pie = [1 0 0 0.025;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];
T_pie_2 = [1 0 0 -0.046;
           0 1 0 0.025;
           0 0 1 0;
           0 0 0 1];
T0{1} = T_temp{1};
T0{2} = T_temp{1}*T_temp{2};
T0{3} = T_temp{1}*T_temp{2}*T_temp{3};
T0{4} = T_temp{1}*T_temp{2}*T_temp{3}*T_temp{4};
T0{5} = T_temp{1}*T_temp{2}*T_temp{3}*T_temp{4}*T_temp{5};
T0{6} = T_temp{1}*T_temp{2}*T_temp{3}*T_temp{4}*T_temp{5}*T_temp{6};
T0{7} = T_temp{1}*T_temp{2}*T_temp{3}*T_temp{4}*T_temp{5}*T_temp{6}*T_pie_2;
%
for i=1:7
    T0M{i} = T_M0*T0{i};
    T0_pie = T_pie;
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
for i=1:8
    T(:,:,i) = T0M{i};
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Marco para dibujar el torso
%Matriz de transformación del marco del torso respecto al marco 5
T4t = [1 0 0 0.1;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
T(:,:,9) = T0M{4}*T4t; %%%%%%%%%%%%%%%%%%%%%%%%Marco del torso en marco 10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Dibujo
% for i=1:7
%     plot([T0M{i}(1,4) T0M{i+1}(1,4)],[T0M{i}(3,4) T0M{i+1}(3,4)],'LineWidth',1.5)
%     axis equal
%     grid on
%     hold on
% end
% hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Jacobiana

