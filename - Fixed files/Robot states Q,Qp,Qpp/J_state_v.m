% Jh (dim20x22) = Jacobiana de h(q), esta NOS SIRVE para calcular las vel. de las operacionales controladas (Qp) DESEADAS [zp,xp_s,xp_s,xp_s,qp_t^T] 
% (donde qp_T =  son las velocidades articulares del torso), a partir de las velocidades de las coordenadas operacionales qp = [qp1... qp21].
% Es decir,  Qp = dh(q)/dq*qp = Qp = JQ*qp.. (aunque aquí le llaman J_h)
function J_h = J_state_v(robot)  
%Computation of the jacobian of h function for virtual constraint
J_h = zeros(robot.joints-1,robot.joints);

%% CoM height
J_G=robot.J_CoM;
J_h(1,:)=J_G(3,:);

%% Foot position
J_h(2,:)=robot.J_Ankle(1,:);
J_h(3,:)=robot.J_Ankle(3,:);
%% Foot pitch
Foot = robot.T(:,:,8)*robot.foot_f;
phi=atan2(Foot(2,1),Foot(1,1));
theta=atan2(-Foot(3,1),cos(phi)*Foot(1,1)+sin(phi)*Foot(2,1));
J_L = zeros(3,robot.joints);
for i = 2:7
    J_L(:,i-1)=robot.T(1:3,3,i); % crea la matriz J_foot,w = [0a2, 0a3, 0a4, 0a5, ..., 0a13, 0 ... 0] (3x22)
end
aux_OMERPY_ffoot = OmeRPY(phi,theta)*J_L;
% J_h(4,:)=aux_OMERPY_ffoot(1,:);
J_h(4,:) = [1 1 1 1 1 1];

%% Hips pitch
Hips = robot.T(:,:,4)*robot.torso_f;
phi=atan2(Hips(2,1),Hips(1,1));
theta=atan2(-Hips(3,1),cos(phi)*Hips(1,1)+sin(phi)*Hips(2,1));
J_T = zeros(3,robot.joints);
for i = 2:4
    J_T(:,i-1)=robot.T(1:3,3,i); % crea la matriz J_torso,w = [0a2, 0a3, 0a4, 0a5, 0a6, 0a7 0 ... 0] (3x22)
end
aux_OMERPY_hips = OmeRPY(phi,theta)*J_T;
% J_h(5,:)=aux_OMERPY_hips(1,:);
J_h(5,:) = [1 1 1 0 0 0];
% Torso pitch
% Torso = robot.T(:,:,5)*robot.torso_f;
% phi=atan2(Torso(2,1),Torso(1,1));
% theta=atan2(-Torso(3,1),cos(phi)*Torso(1,1)+sin(phi)*Torso(2,1));
% J_Torso = zeros(3,robot.joints);
% for i = 2:4
%     J_Torso(:,i-1)=robot.T(1:3,3,i);  % ....
% end
% J_Torso(:,4)=robot.T(1:3,3,5);      % crea la matriz J_UpperTorso,w = [0a2, 0a3, 0a4, 0a5, 0a6, 0a7, 0a15 0 ... 0] (3x31)
% Ome = OmeRPY(phi,theta);
% J_h(6,:)=Ome(1,:)*J_Torso;
end

