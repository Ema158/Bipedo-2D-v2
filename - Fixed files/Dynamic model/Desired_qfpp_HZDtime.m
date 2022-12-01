function [qfpp, JPhi, JPhipPhip, NE1, NE3, qfp] = Desired_qfpp_HZDtime(ZMPd,robot_i,X,gait_param_i,t)

% Este programa utiliza parte del código de los siguientes programas:
% - joints_Plus03.m - > para el cálculo de "JPhi = JQ^{-1}Jd" y "gf = Jdp*qpf - JQp*qp"

ZMPd_x = ZMPd(1);

global robot gait_parameters
robot = robot_i;
gait_parameters = gait_param_i;

% states
x = X(1);
xp =X(2);

% =======================================================================
%   Cálculo de JPhi y JPhipPhi
% -----------------------------------------------------------------------
qf = x;
qfp = xp;
[q,qD,JPhi,JQ,~] = JointsPosVel_from_CoMPosVel_HZDtime(qf,qfp,robot,gait_parameters,t);
robot = robot_move(robot,q);

robot.qD = qD; % Updating of the robot velocities since they will be used for the computing of several subsequent matrices
JQpqp = JQp_qp02(robot,JQ);

Case = 'acceleration';
% "Case", "gait_parameters", "qfp" and "t" are used inside "OptionDesiredTrajectory"
OptionDesiredTrajectory;
% ------------------------------
JdpPhip = [dt_dhd_Phi; 
          zeros(1,1+phiDim)]*Phip;
            
JPhipPhip = JQ\(JdpPhip - JQpqp);

% ====================================================================================
% Cálculo de pares y fuerza y momento de reacción para PARTE de la acceleración qpp.
% ------------------------------------------------------------------------------------
% Se toma en cuenta que
% qpp = Jf*qpp + gf = Jf(col1)*xpp + Jf(col2)*ypp  + gf.. entonces, se calculan las pares y fuerzas para
%  "Jf(col1) + gf", "Jf(col2)+ gf"  y "gf" de tal forma que podamos despejar y encontrar "qfpp"
% -----------------------------------------------------------------------
% Utilizando la OPCION III utilizada en "RomeoNewtonEuler02.m"  (Se usa el algoritmo de NE 3 VECES, -opción 2 en el reporte-)
% -----------------------------------------------------
term1 = JPhi(:,1) + JPhipPhip; %
term3 = JPhipPhip; %
[F1,M1,Tau1]= Newton_Euler(q,qD,term1);  % [F1 M1 Tau1]' = De*(Jf1 + gf) + He
[F3,M3,Tau3]= Newton_Euler(q,qD,term3);  % [F3 M3 Tau3]' = De*gf + He
% =================================================================================================
% Si quisiéramos encontrar la fuerza F momento M y pares Tau reales debido a q, qp y qpp, deberíamos utilizar qppf de tal
% forma que 
%  F = (F1-F3)*xpp + (F2-F3)*ypp + F3 ;  
%  M = (M1-M3)*xpp + (M2-M3)*ypp + M3 ;  
%  Tau = (Tau1-Tau3)*xpp +  (Tau2-Tau3)*ypp + Tau3;
% Sin embargo lo que queremos encontrar es la trayectoria de "qfpp" que hará que el ZMPd = ZMP.
% AHORA, usando F0a despejaremos "xpp" y "ypp" y luego lo compararemos con XDD
% ---------------------------------------------------------------------------
%  vect1 = [F1 - F3; M1 - M3; Tau1 - Tau3]; % Vector de 37 x 1
%  vect2 = [F2 - F3; M2 - M3; Tau2 - Tau3]; % Vector de 37 x 1
%  vect3 = [F3; M3; Tau3]; % Vector de 37 x 1
%  vect0 = [F; M; Tau]; % Estos son los pares y fuerzas completos obtenidos con NE(q,qp,qpp)
%  Jfe = [vect1 vect2]; % Matriz de 37 x 2
%  XDDn = inv(Jfe'*Jfe)*Jfe'*(vect0 - vect3); 

% =======================================================================
%   Cálculo de qfpp
% -----------------------------------------------------------------------
NE1 = [F1;M1;Tau1];
NE3 = [F3;M3;Tau3];

DeBar = NE1-NE3;
HeBar = NE3;
% Matriz 2x2
DrBar = DeBar(5,:) + ZMPd_x*DeBar(3,:); 
% Vector 2x1
HrBar = HeBar(5) + ZMPd_x*HeBar(3);     
% Aceleración xpp y ypp "deseada"
qfpp = -DrBar\HrBar;
