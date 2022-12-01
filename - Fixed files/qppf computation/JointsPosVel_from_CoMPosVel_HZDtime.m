function [q,qp,JPhi,JQ,Jd] = JointsPosVel_from_CoMPosVel_HZDtime(qf,qfp,robot,gait_parameters,t)
% Computation of Joints positions and velocities from CoM position and VELOCITIES
x = qf(1);
% Posiciones
%-----------------
q = InvGeometricHZDtime(x,robot,gait_parameters,t);
%-------------------------
robot = robot_move(robot,q);

Case = 'velocity';
% "Case", "gait_parameters", "qfp" and "x" are used inside "OptionDesiredTrajectory"
OptionDesiredTrajectory;

% Velocities
%-----------------
%Jacobian (dQ/dq)
JQ = [J_state_v(robot);robot.J_CoM(1,:)];
%Matrix Jd
Jd = [dhd_Phi;    
        1, zeros(1,phiDim)];
JPhi = JQ\Jd;

%-------------------------
qp = JPhi*Phip;
%-------------------------