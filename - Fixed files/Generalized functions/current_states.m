function [Q, Qp] = current_states(robot)
% This function give the current states of the robot Q -> 31x1 and Qp-> 31x1 where Q is given by
% Q =   [   z       	 % Position in z of the CoM 
%         x_foot         % Position in x of the free foot. Obtained from frame 8
%         z_foot         % Position in z of the free foot. Obtained from frame9 
%         theta_foot     % Pitch orientation. Rotation in y of the free foot. Obtained from frame 9
%         theta_hip    % Pitch orientation. Rotation in y of the torso. Obtained from frame4 
%         theta_torso %Pitch orientation.Rotation in y ofthetorso.Obtained from fram5
%           x]           % Position in x of the CoM 
% and Qp = dQ/dt - > are the velocities.

% -----------------------
% Positions: Q = f(q) = [h(q); x; y]
% ------------------
G = robot.CoM;  % Posición X,Y,Z del COM del robot
Q = [state_v(robot);G(1)];  %

%-------------------
% Velocities Qp = JQ(q)*qp ...where JQ = df(q)/dq
%-----------------
%Jacobian (dQ/dq)
JQ = [J_state_v(robot);robot.J_CoM(1,:)];
Qp =  JQ*robot.qD;

