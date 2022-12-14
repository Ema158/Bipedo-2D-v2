
function [t,Xt,t_SS,Xt_SS,t_DS,Xt_DS,t_DS_final,Xt_DS_final,gait_parameters_SS,gait_parameters_DS,gait_parameters_DS_final,gait_parameters_gait] = robot_step_EssModel_Start_DS_SS_DS_gait_t(gait_parameters_SS,gait_parameters_DS,gait_parameters_DS_final,gait_parameters_gait,N)

% ==============================================================================================
% Options
% ----------------------------------------------------------------------------------------------
% NOTE The number of iterations performed by the solver are counted by "contB". This is used to stop the solver if the
% robot doesn't perform a step after a determinated number of iterations (this is done in "PEventsHZDtime")
global contB 
contB =  1;            % To count the number of iterations performed by the PEvents file (recommended)
% global DisplayIterNumber % To show how many iteration have been performed by the solver 
% DisplayIterNumber = []; % To DISPLAY the information of the number of iteration performed by the solver.. Empty-> Not display anything
% ==============================================================================================
global contD OutOfWorkSpace
contD = []; % Every time the robot is out of the workspace and "PEvents" is called, 'contD' is used to stop de integration 
OutOfWorkSpace = []; % Reset workspace flag

global robot gait_parameters
%% DS phase
% ==========================================================
gait_parameters = gait_parameters_DS;
T = gait_parameters.T;
disp('DS phase')
disp('--------------------------')
% Initial states
x0 = 0;
xp0 = 0;
X0_DS = [x0,xp0];

% Double Support (SS) Phase
options = odeset('Events', @PEvents_HDZtimeDS,'RelTol', 1.e-7, 'AbsTol', 1.e-9);
[t_DS,Xt_DS] = ode45(@dynam_HZDtime,0:1e-3:T,X0_DS,options);

xf_DS = Xt_DS(end,1);
xpf_DS = Xt_DS(end,2);
%% Single Support (SS) Phase
gait_parameters = gait_parameters_SS;
T = gait_parameters.T;
% Restart iteration counter
contB = 1;
disp('SS phase')
disp('--------------------------')
% Initial states for SS phase
x0 = xf_DS;
xp0 = xpf_DS;
X0_SS = [x0,xp0];
% In PEventsDS the high of the foot is not checked.. but since the gait is defined by a constant desired time T, that
% condition is not SO important
options = odeset('Events', @PEvents_HDZtimeDS,'RelTol', 1.e-7, 'AbsTol', 1.e-9);
[t_SS,Xt_SS] = ode45(@dynam_HZDtime,0:1e-3:T,X0_SS,options);

xf_SS = Xt_SS(end,1);
xpf_SS = Xt_SS(end,2);
%% DS final phase
% ==========================================================
gait_parameters = gait_parameters_DS_final;
T = gait_parameters.T;
disp('DS phase (final)')
disp('--------------------------')
% Initial states for SS phase
x0 = xf_SS;
xp0 = xpf_SS;
X0_DS_final = [x0,xp0];

% Double Support (SS) Phase
options = odeset('Events', @PEvents_HDZtimeDS,'RelTol', 1.e-7, 'AbsTol', 1.e-9);
[t_DS_final,Xt_DS_final] = ode45(@dynam_HZDtime,0:1e-3:T,X0_DS_final,options);

xf_DS_final = Xt_DS_final(end,1);
xpf_DS_final = Xt_DS_final(end,2);
% figure(5)
% robot_draw(robot,0,0)             % Supported on the right foot
% view(3) % to assigne a "standard" view in 3D
% axis equal 
%%  Gait phase
% ================================================================
gait_parameters = gait_parameters_gait;
T = gait_parameters.T;
S = gait_parameters.S;
g=9.81;
z0 = gait_parameters.z_i;
disp('Gait phase')
disp('--------------------------')
% Initial states for SS phase
x0 = xf_DS_final;
xp0 = xpf_DS_final;

% Double Support (SS) Phase
SupportFootX = 0;
ZMPd = cell(1,N+1);
%% Continue...
omega = sqrt(g/z0); % the units are:  sqrt((m/s^2)/m) = 1/s
ZMPxCoeff = gait_parameters.ZMPxCoeff;
t = cell(1,N);
Xt = cell(1,N); 
Ex = cell(1,N); 
Ey = cell(1,N); 
L = cell(1,N);
zt = cell(1,N);
zpt = cell(1,N);
SupportFoot = cell(1,N);
for j=1:(N)    
    SupportFoot{j} = SupportFootX;
    
    % ====================================================================
    % Computation of one step of the robot: 
    % Previous states before impact -> the Impact and Rellabelling -> Evolution of one step -> states before impact
    % ====================================================================
    X_final = [x0;xp0]; % previous step
%     Q = state_v(robot);
    [t{j},X0,Xt{j}] = robot_step_EssModel_t(X_final); % "robot" and "gait_parameters" structures are needed inside
    % Initial (current step) after impact
    x0 = X0(1); 
    xp0 = X0(2);
    % Final states:
    xt = Xt{j}(:,1);   % Solution of position in X of the CoM for step j
    xpt = Xt{j}(:,2);  % Solution of velocity in X of the CoM for step j
    % -----------------------------------------------------------------------------------------------------------------
    
    % IMPORTANT: IN THIS PART of the CODE IS JUST FOR VISUALIZATION for the trajectory of the Desired ZMP.
    %            The REAL evolution of the ZMP will be obtained by runing code "M20_..." =)
    % Evolution of the ZMP 
    samples = length(t{j});
    ZMPdX = zeros(1,samples);
    for i=1:samples
        ZMPdX(i) = polyval(ZMPxCoeff,t{j}(i));
        zt{j}(i) = polyval(gait_parameters.PolyCoeff.hd1,t{j}(i));
        zpt{j}(i) = polyval(polyder(gait_parameters.PolyCoeff.hd1),t{j}(i));
    end   
    ZMPd{j}(1,:) = SupportFootX + ZMPdX;
    % ========================================================
    
    % Orbital energy % It's always measured in the LOCAL frame (this is just for the LIP but it helps to see if the step is periodic)
    % --------------------------------------------
    Ex{j} = -(g/(2*z0)).*xt.^2 + (1/2).*xpt.^2; % Orbital energy in X
    
    % Synchonization measure (this is just for the LIP but it helps to see if the step is periodic)
    % --------------------------------------------    
%     L{j} = xpt.*ypt - omega^2*xt.*yt;
    
    % Final states:
    x0 = xt(end);    % Final position in X of the CoM for step j
    xp0 = xpt(end);  % Final velocity in X of the CoM for step j
    
    % Display information
    % ----------------------------------------------------    
    fprintf('Step %d of %d\n',j,N);   
    fprintf('Initial ZMP position in X: %0.3f\n',ZMPd{j}(1,1));
    fprintf('Final ZMP position in X: %0.3f\n',ZMPd{j}(1,end));
    fprintf('Initial states: [x0 xp0] = [%f,%f]\n',xf_DS_final,xpf_DS_final);
    fprintf('Final states: [xf xpf] = [%f,%f]\n',x0,xp0);
    disp('-------------------------------------------');
    % Initial conditions for the Next step
    %-------------------------                    
    SupportFootX = SupportFootX + S;
    contB = 1; % Reinitializing the counter of the solver
%     nj=S*j;
%     if mod(j,2)
%     figure(2)
%     plot(Xt{1,j}(:,1) + nj,Xt{1,j}(:,2));
%     hold on
%     else
%     figure(2)
%     plot(Xt{1,j}(:,1) + nj,Xt{1,j}(:,2));
%     hold on
%     end
%     figure(6)
%     robot_draw(robot,0,0)             % Supported on the right foot
%     view(3) % to assigne a "standard" view in 3D
%     axis equal 
end

%% Reinitialization if needed...
% Computing of the joints velocities and positions at the end of the step
if ~isempty(OutOfWorkSpace) % If the CoM of the robot is always inside the workspace of the robot....    
    disp('Robot configuration is OUT OF WORKSAPCE at the end of the step in "robot_step...m". Robot configuration RE-Initialized')
    robot = genebot();     
end 

global noLanding
if ~(isempty(noLanding) || noLanding==0) % if there was no impact
    disp('Robot configuration is INACCESSIBLE in "robot_step...m". Robot configuration RE-Initialized')
    robot = genebot();  
end

