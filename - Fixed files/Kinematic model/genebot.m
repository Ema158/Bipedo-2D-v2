function robot = genebot()
%%Generation of the robot datas

%% Number of joints
joints = 6;

%% ZERO joint positions
q = zeros(joints,1); 

%% Joint velocities
qD = zeros(joints,1);

%% Creating the robot structure
robot = struct('joints',joints,'q',q,'qD',qD);

%% CONSTANT transformation matrices to convert make all matrices 0Ti be aligned with the world frame (frame 0) at ZERO position
% ------------------------------------------
robot.q = q;
robot.T = DGM(robot);
Tconst = zeros(4,4,9);
for i=1:8
    Tconst(:,:,i) = eye(4);
    R = robot.T(1:3,1:3,i);
    Tconst(1:3,1:3,i) = R';
end    
robot.Tconst = Tconst;

%% Assigning DEFAULT joint positions
q(1)=0.1;
q(2)=-0.4;
q(3)=0.4;
q(4)=-0.4;
q(5)=0.4;
q(6)=-0.1;
%
% q(1)=0;
% q(2)=-0;
% q(3)=0;
% q(4)=-0;
% q(5)=0;
% q(6)=0;
%
robot.q = q;
robot.T = DGM(robot); % Modelo geométrico directo (calcula las matrices de transformación elementales del robot),
                       % y las asigna a la estructura "robot" en la variable T.                       

%% Robot mass information
PI = Mass_information;
M = PI.masse;
robot.mass = sum(M);
robot.PI = PI;

[robot.CoM,robot.J_CoM,robot.J_Ankle,robot.crossM,robot.J_CoMs] = compute_com(robot,PI);

%% Ankle frame (8) to world frame
robot.foot_f = [  0 1 0 0;
                  0 0 1 0;
                  1 0 0 0;
                  0 0 0 1];
% robot.foot_f = [  0 0 1 0;
%                   0 -1 0 0;
%                   1 0 0 0;
%                   0 0 0 1];
%% Hip frame (4) to world frame
robot.torso_f=[0     1   0   0;
               0     0   1   0;
               1     0   0   0;
               0     0   0   1];
   
%% Robot antecedent and joint list
robot.ant = [0,... 1
             1,... 2
             2,... 3
             3,... 4
             4,... 5
             5,... 6
             6,... 7
             7]; %8
robot.act= [0,1,2,3,4,5,6,0];

end

