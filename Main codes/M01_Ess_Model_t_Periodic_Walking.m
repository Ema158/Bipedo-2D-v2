close all;
clear 
clc
% ---------------------------------------------------------------------------------------------
currentfolder = pwd; % save current path
cd ..                % go one folder down (go out of the current folder and stay in the previous one)
add_paths;           % add all folders where all files are founded in order to acces to them
cd(currentfolder);   % return to the original path
% ---------------------------------------------------------------------------------------------
echo on
% Testing Essential dynamics in a planar robot
% ==============================
% Creation: 01/12/2022
% Last modification: -/--/--
% --------------------------------------------------------------------------------------------------
% Periodic Walking of a Essential model of a 2D biped robot.
% -----------------------------------------------------------------------------------------------------
% The difference is that in here the same file for compute the Coefficients for the polyynmomials to build the desired
% trajectories of the controlled variables is used for the desired values without impact and with impact.
% Also a trajectory for the twist the upper torso is builded.
% No periodic motion was intented to found for the inclusion of the upper body motion since it doesn't affect so much.
% So the same periodic motion founded for time with impact was used
% --------------------------------------------------------------------------------------------------
% 
%
echo off

global gait_parameters
global robot 
% Parameters
% ---------------------------------------------------------
Biped_param = Param01_2DBiped_PeriodicGait_ZMP0();
% ---------------------------------------------------------
gait_parameters = Biped_param.gait_parameters;
T = gait_parameters.T;
g = gait_parameters.g;
z0 = gait_parameters.z_i;
S = gait_parameters.S;

% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
global OptionContVar  % Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_t" and "hppd_Polyn_t". 
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t". 
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x". 
OptionContVar = Biped_param.ControlledVariableOption;
% -------------------------------------------------------------------------------------------------

% GENERAL OPTIONS
% ----------------------------------------------------
DataName = 'InfoBiped_Param01_PeriodicGait';
Animation = true;  % Do you want to show the animation?
n = 8; % number of SAMPLES for each step of the animation
LineWidth = 2;  % For plots and animation
FontSize = 14;  % For plots and animation 
produceVideo = true;   % Do you want to produce a video? (if animation if false no video will be produced)
framerate = (n/T)/3; % with n/T the real time duration of the video will be T, the last divition will produce
                    %  a duration of video "factor*T". For example if T=0.5, " (n/T)/3" will produce a video of 1.5 sec 
% ----------------------------------------------------
N = 4; % number of steps
% Initial support for the first step
SupportFootX = 0;
ZMPd = cell(1,N+1);

% Final conditions for the step 0
% Cyclic motions:
% -------------------------------------------------------------------------------------------------------------------
Rcyc = Biped_param.Rcyc;

% If NO error in the velocity is desired, chose "Error = 0"
Error = 0.0; % Percentage of error   <-------
if Error ~= 0
    fprintf('Motion started OUTSIDE the periodic motion, with an error in velocity of %.2f %%\n',Error*100)
    disp('-----------------------------------------------------------------------------------')
end
% ----------------------------------
Dx = Rcyc(1);
xpf = Rcyc(2);
% Initial and final position is computed as 
% x0 = -S/2 + Dx;
xf = S/2 + Dx;

% Display information
% ----------------------------------------------------
disp('Information for each step');
disp('==========================');
fprintf('Step length S: %d\n',S);
fprintf('Step time T: %d\n',T);
disp('-------------------------------------------');

%% Parameteres and initialization of ROMEO
% ======================================================================================
global alphaM % alphaM = [0-1]. Allow to remove gradually the value of all masses and inertias of the robot
% alphaM = 0 -> all masses are 0 -> all mass is concentrated in one point
% alphaM = 1 -> all masses are normal -> all masses are distributed
alphaM = 1;
if alphaM == 0
    fprintf('All mass concentred in ONE point (as a 3DLIP), i.e. alphaM = %f\n',alphaM)
elseif alphaM == 1
    fprintf('All masses distributed (as it should be), i.e. alphaM = %f\n',alphaM)
else
    fprintf('Masses distributed taking into account alphaM = %f . If alphaM -> 0 all masses will be concentred in ONE point\n',alphaM)
end
% ------------------------------------------------------------------------------------

% Generating the robot structure
robot = genebot();

% Computation of the initial coefficients the polinomial trajectories for the controlled variables. 
% These coefficients are changed inmediately after the transition (really necessary when there is an impact)
gait_parameters.transition = false; 
PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters);
gait_parameters.PolyCoeff = PolyCoeff;

global DisplayIterNumber  % To show how many iteration have been performed by the solver 
DisplayIterNumber = 1;

%% Continue...
omega = sqrt(g/z0); % the units are:  sqrt((m/s^2)/m) = 1/s
ZMPxCoeff = gait_parameters.ZMPxCoeff;
t = cell(1,N);
Xt = cell(1,N); 
Ex = cell(1,N); 
L = cell(1,N);
zt = cell(1,N);
zpt = cell(1,N);
SupportFoot = cell(1,N);
fprintf('Initial states final states of the CoM: \n[xf xpf] = [%f,%f]\n',xf,xpf);
disp('-------------------------------------------');
for j=1:N    
    SupportFoot{j} = SupportFootX;
    
    % ====================================================================
    % Computation of one step of the robot: 
    % Previous states before impact -> the Impact and Rellabelling -> Evolution of one step -> states before impact
    % ====================================================================
    X_final = [xf;xpf]; % previous step
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
    xf = xt(end);    % Final position in X of the CoM for step j
    xpf = xpt(end);  % Final velocity in X of the CoM for step j
    
    % Display information
    % ----------------------------------------------------    
    fprintf('Step %d of %d\n',j,N);   
    fprintf('Initial ZMP position in X: %0.3f\n',ZMPd{j}(1,1));
    fprintf('Final ZMP position in X: %0.3f\n',ZMPd{j}(1,end));
    fprintf('Initial states: [x0 xp0] = [%f,%f]\n',x0,xp0);
    fprintf('Final states: [xf xpf] = [%f,%f]\n',xf,xpf);
    disp('-------------------------------------------');
    % Initial conditions for the Next step
    %-------------------------                    
    SupportFootX = SupportFootX + S;
    contB = 1; % Reinitializing the counter of the solver
end

%% =====================================
% Plots
% =============
% States for all the trajectory
xtAll = [];
ztAll = [];
xptAll = [];
tAll = [];
ExAll = [];
LAll = [];

% Sampled states for all the trajectory
xsAll = [];
ysAll = [];
zsAll = [];
xpsAll = [];
ypsAll = [];
tsAll = [];

% To determine the maximum and minimum value of the CoM position to determine the dimension of the animation of the plot
% ------------------------------------------------------------------------------------------------------------
for j=1:N
   maxPosJx(j) = SupportFoot{j} + max(Xt{j}(:,1)); % max position in x for the step "j"
   maxPosJx(j) = max(maxPosJx(j),SupportFoot{j});
   minPosJx(j) = SupportFoot{j} + min(Xt{j}(:,1));
   minPosJx(j) = min(minPosJx(j),SupportFoot{j}); 
end
maxPosX = max(maxPosJx);
minPosX = min(minPosJx);
% ------------------------------------------------------------------------------------------------------------

% Building of trajectories for all the steps and animation
% ---------------------------------------------
if Animation && produceVideo
    writerobj = VideoWriter(strcat(['ReducedModel_',DataName],'.avi'));
    writerobj.FrameRate = framerate; % The smaller FrameRate the slower the video (Frames per second)
    open(writerobj);    
    disp('Making a video...')    
end
ZMPdS = cell(1,N);
for j=1:N    
    xt = SupportFoot{j} +  Xt{j}(:,1)';  % global position of x(t) for step "j"
    xpt = Xt{j}(:,2)';                       % xp(t) for step "j"     
    tG = (j-1)*T  + t{j}';  % tG is the global time for all the steps
    
    % ===============================================================
    %     Variables for animation
    % ===============================================================
    if Animation                
        % Sampled variables
        OnesRowVectS = ones(1,n);
        Xs = sampling(Xt{j}',n); % Xs is the sampled evolution of the CoM  for step "j" in the LOCAL frame attached to the support foot
        ts = (j-1)*T  + sampling(t{j}',n);  % ts is the sampled time for all the steps
        ZMPdS{j} = sampling(ZMPd{j},n);  % ZMPdS is the sampled ZMP for each step;
        
        xs = SupportFoot{j} + Xs(1,:); % Sampled position of the CoM in X for step "j" in the WORLD frame
        xps = Xs(2,:);                      % Sampled velocity in X of the CoM for step "j"
        zs = sampling(zt{j},n); % Xs is the sampled evolution of the CoM  for step "j" in the LOCAL frame attached to the support foot        
        % zs = z0*OnesRowVectS;
        
        % Colors for plots % color = [R,G,B];
        % ------------------------------------
        color = rand(1,3); % Random Color
        
        xsAll = [xsAll xs];
        zsAll = [zsAll zs];
        CoMIniEndPos{j} = [xs(1),zs(1),xs(end),zs(end)];
        for i=1:n
            fig = figure (1);
            % if we want to draw all the trace of the pendulum we must put it "on"
            hold off
            plot([ZMPdS{j}(1,i),xs(i)],[0,zs(i)],'-ko');
            hold on
            % The next part draw the initial and final position of the pendulum for each step
            % ------------------------------------------------------------------------------------
            for k=1:j
                % Initial position
                plot([ZMPd{k}(1,1),CoMIniEndPos{k}(1,1)],[0,CoMIniEndPos{k}(1,2)],'--o', 'color',[0.68,0.47,0]);
                % Final position
                if k~=j % It doesn't draw the final position of the pendulum until the next step
                    plot([ZMPd{k}(1,end),CoMIniEndPos{k}(1,3)],[0,CoMIniEndPos{k}(1,4)],'--o', 'color',[0.48,0.06,0.89]);
                end
                % Trajectory of the ZMP                
                if k<j
                    plot(ZMPdS{k}(1,:),0*OnesRowVectS,'LineWidth',LineWidth);
                else
                    plot(ZMPdS{k}(1,1:i),0*(1:i),'LineWidth',LineWidth);
                end
            end
            % Trajectory of the CoM
            plot(xsAll(1:i+(j-1)*n),zsAll(1:i+(j-1)*n),'color',color,'LineWidth',LineWidth)
            axis([minPosX-0.001,maxPosX+0.001,0,0.15]); % axis([x0,xf,y0,yf,z0,zf]);
            xlabel('x [m]','interpreter','Latex','FontSize',FontSize);
            ylabel('z [m]','interpreter','Latex','FontSize',FontSize);
            title(['t = ' num2str(ts(i)) ' [sec]']);
            grid on
            if  produceVideo
                writeVideo(writerobj,getframe(fig));
            end 
            pause(0.5)
        end        
        % Sampled states for all the trajectory
        xpsAll = [xpsAll xps];
        tsAll = [tsAll ts];
        
    end
    % ===============================================================
    % States for all the trajectory
    xtAll = [xtAll xt];
    xptAll = [xptAll xpt];  
    tAll = [tAll tG];
    ExAll = [ExAll Ex{j}'];                  % ExAll is the orbital energy in X for all the steps
    LAll = [LAll L{j}'];                     % LAll is the synchronization measure for all the steps
end
if Animation && produceVideo
    close(writerobj);
    disp('Video produced successfully')
    disp('------------------------------')
end

%% Storing all the parameters and information of this simulation
% =====================================================================
Solution.t = t;
Solution.Xt = Xt;
Solution.XtAll = [tAll', xtAll', xptAll'];
if Animation
    Solution.XSampledAll = [tsAll', xsAll', xpsAll'];
end
Solution.SupportFootPos = SupportFoot;
Solution.zt = zt;
% Build just ONE structure with all the information
InfBipedo.robot = robot;
% % InfNAO.iniParameters = initParam;
InfBipedo.gait_parameters = gait_parameters;
InfBipedo.Solution = Solution;
disp(['Saving data as: ' DataName]);
disp('------------------------------');
save(DataName,'InfBipedo')

%% =====================================================================
% Evolution of the position and velocity CoM in X w.r.t. time
% ---------------------------------------------
figure (2)
subplot(3,1,1)  
hold on
plot(tAll,xtAll,'color','k','LineWidth',LineWidth)
axis([min(tAll) max(tAll) -inf inf])
xlabel('$t$ [s]','interpreter','Latex','FontSize',FontSize);
ylabel('$x(t) [m]$','interpreter','Latex','FontSize',FontSize);
subplot(3,1,2)  
hold on
plot(tAll,xptAll,'color','k','LineWidth',LineWidth)
axis([min(tAll) max(tAll) -inf inf])
xlabel('$t$ [s]','interpreter','Latex','FontSize',FontSize);
ylabel('$\dot{x}(t)$ [m/s]','interpreter','Latex','FontSize',FontSize);
subplot(3,1,3)  
hold on
plot(tAll,ExAll,'color','k','LineWidth',LineWidth)
axis([min(tAll) max(tAll) -inf inf])
xlabel('$t$ [s]','interpreter','Latex','FontSize',FontSize);
ylabel('$E_x(t)$ ','interpreter','Latex','FontSize',FontSize);

%% End of the code
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------