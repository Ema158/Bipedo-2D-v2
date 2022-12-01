% This file compute the term JQp*qp (31x1) which is used to calculate "qpp" from Qpp = JQ*qpp + JQp*qp, i.e. 
% qpp = inv(JQ)[Qpp - JQp*qpp]
function JQpqp = JQp_qp02(robot,JQ)  
%Computation of the derivative of jacobian of f function with respect time
% In here, computation is based on "VelPartialAccCoMs_Frames.m" and NOT in Symoro =)
% q = robot.q;
qp = robot.qD;

JQpqp = zeros(robot.joints,1);
% Obtenemos las Jp*qp de los centros de masa y de los marcos
%[~, ~, JpCoMqp, ~, Jpi_qp] = VelPartialAccCoMs_Frames(robot);
 [~, ~, JpCoMqp, ~, Jpi_qp] = VelAceCoMs_Frames(robot,zeros(6,1));% Podemos utilizar tambien ésta función usando qpp = 0 =)

H4 = Jpi_qp{4};    % [^0PartialVP_7; ^0PartialWP_7] = [^0Jp_4,v*qp; ^0Jp_4,w*qp]
H7 = Jpi_qp{7};  % [^0PartialVP_12; ^0PartialWP_12] = [^0Jp_8,v*qp; ^0Jp_8,w*qp]
H8 = Jpi_qp{8};  % [^0PartialVP_14; ^0PartialWP_14] = [^0Jp_9,v*qp; ^0Jp_9,w*qp]

%% CoM height
JQpqp(1)= JpCoMqp(3); % The row corresponding to zpp of the CoM


%% Foot 
JQpqp(2:3)= [H7(1);H8(3)]; % [H12(1,2);H14(3)] = J_foot,v*qp

%% Foot pitch
Foot = robot.T(:,:,9)*robot.foot_f;
phi = atan2(Foot(2,1),Foot(1,1));
theta = atan2(-Foot(3,1),cos(phi)*Foot(1,1)+sin(phi)*Foot(2,1));
thetap = JQ(4,:)*qp;
phip = 0;
Omega = OmeRPY(phi,theta);
OmegaDot = OmeDotRPY(phi,theta,phip,thetap);
% Recalling that, as we just computing JQp*qp the first term i.e. JQ*qpp is computed outside
% Note that "JQ(5:7,:) = Omega*Jfoot,w"... and for the second term (first term here) we want: "OmegaDot*J_foot,w *qp"
JQ_aux = [zeros(1,6);JQ(4,:);zeros(1,6)];
term2 = OmegaDot*inv(Omega)*JQ_aux*qp;  % OmegaDot*J_foot,w *qp
H9_aux = [0;H8(5);0];
term3 = Omega*H9_aux;                    % Omega*Jp_foot,w *qp 
sum_term = term2 + term3;
JQpqp(4) = sum_term(2); 

%% Hips pitch (lower Torso)
Hips = robot.T(:,:,4)*robot.torso_f; % Since frame 7 in zero potition (i.e. q=0 ) has a diferent orientation as frame 0. 
                    % It must be oriented as frame 0, so that the pitch yaw and roll angles have the same meaning for the
                    % torso and for the foot.                    
phi=atan2(Hips(2,1),Hips(1,1));
theta=atan2(-Hips(3,1),cos(phi)*Hips(1,1)+sin(phi)*Hips(2,1));
thetap = JQ(5,:)*qp;
phip = 0;
Omega = OmeRPY(phi,theta);
OmegaDot = OmeDotRPY(phi,theta,phip,thetap);
% Recalling that, as we just computing JQp*qp the first term i.e. JQ*qpp is computed outside
% Note that "JQ(8:10,:) = Omega*Jtorso,w"... and for the second term (first term here) we want: "OmegaDot*J_torso,w *qp"
JQ_aux = [zeros(1,6);JQ(5,:);zeros(1,6)];
term2 = OmegaDot*inv(Omega)*JQ_aux*qp;  % OmegaDot*J_torso,w *qp
H4_aux = [0;H4(5);0];
term3 = Omega*H4_aux;                      % Omega*Jp_torso,w *qp   
sum_term = term2 + term3;
JQpqp(5) = sum_term(2); 

%% Torso pitch
% Torso = robot.T(:,:,5)*robot.torso_f; % Since frame 7 in zero potition (i.e. q=0 ) has a diferent orientation as frame 0. 
%                     % It must be oriented as frame 0, so that the pitch yaw and roll angles have the same meaning for the
%                     % torso and for the foot.                    
% phi=atan2(Torso(2,1),Torso(1,1));
% theta=atan2(-Torso(3,1),cos(phi)*Torso(1,1)+sin(phi)*Torso(2,1));
% thetap = JQ(6,:)*qp;
% phip = 0;
% Omega = OmeRPY(phi,theta);
% OmegaDot = OmeDotRPY(phi,theta,phip,thetap);
% % Recalling that, as we just computing JQp*qp the first term i.e. JQ*qpp is computed outside
% % Note that "JQ(8:10,:) = Omega*Jtorso,w"... and for the second term (first term here) we want: "OmegaDot*J_torso,w *qp"
% JQ_aux = [zeros(1,6);JQ(6,:);zeros(1,6)];
% term2 = OmegaDot*inv(Omega)*JQ_aux*qp;  % OmegaDot*J_torso,w *qp
% H5_aux = [0;H5(5);0];
% term3 = Omega*H5_aux;                      % Omega*Jp_torso,w *qp   
% sum_term = term2 + term3;
% JQpqp(6) = sum_term(2); 

JQpqp(6)= JpCoMqp(1); % The row corresponding to xpp of the CoM


end

