close all;
clear all;
clc
% ---------------------------------------------------------------------------------------------
currentfolder = pwd; % save current path
cd ..                % go one folder down (go out of the current folder and stay in the previous one)
add_paths;           % add all folders where all files are founded in order to acces to them
cd(currentfolder);   % return to the original path
% ---------------------------------------------------------------------------------------------

tic;  % Starting of counting the elapsed time of this function.
% GENERAL OPTIONS
% ----------------------------------------------------
DataName = 'InfBiped_Param01_Start_Gait_SS_Stop';
% ----------------------------------------------------
diary(['- Bipedlog-',DataName,'.txt']) % Create a .txt file to store all display information
diary on % Start the recording of the log file
echo on

% Programa que ejecuta la simulacion de la fase inicial y caminado de n
% pasos
echo off
global robot

% Parameters
% ---------------------------------------------------------
%
Biped_param_DS = Param01_2DBiped_StartingDS();
Biped_param_SS = Param01_2DBiped_StartingSS();
Biped_param_DS_final = Param01_2DBiped_StartingDS_final();
Biped_param_gait = Param01_2DBiped_PeriodicGait_ZMP0();
Biped_param_Stop_SS = Param01_2DBiped_StoppingSS();
Biped_param_Stop_DS = Param01_2DBiped_StoppingDS();
% ---------------------------------------------------------
global gait_parameters_SS gait_parameters_DS gait_parameters_DS_final gait_parameters_gait_SS 
global gait_parameters_SS_Stopping gait_parameters_DS_Stopping
gait_parameters_SS = Biped_param_SS.gait_parameters;
gait_parameters_DS = Biped_param_DS.gait_parameters;
gait_parameters_DS_final = Biped_param_DS_final.gait_parameters;
gait_parameters_gait_SS = Biped_param_gait.gait_parameters;
gait_parameters_SS_Stopping = Biped_param_Stop_SS.gait_parameters;
gait_parameters_DS_Stopping = Biped_param_Stop_DS.gait_parameters;
%
S = Biped_param_gait.gait_parameters.S;
T = Biped_param_gait.gait_parameters.T;
Nstep=3;
% GENERAL OPTIONS
% ----------------------------------------------------
Animation = true;  % Do you want to show the animation?
n = 8; % number of SAMPLES for each step of the animation
LineWidth = 2;  % For plots and animation
FontSize = 14;  % For plots and animation 
produceVideo = true;   % Do you want to produce a video? (if animation if false no video will be produced)
framerate = (n/T)/3; % with n/T the real time duration of the video will be T, the last divition will produce
                    %  a duration of video "factor*T". For example if T=0.5, " (n/T)/3" will produce a video of 1.5 sec 
% ----------------------------------------------------
% Initial support for the first step
SupportFootX = 0;
ZMPd = cell(1,Nstep+1);
% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
global OptionContVar  % Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_t" and "hppd_Polyn_t". 
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t". 
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x". 
OptionContVar = 1;
% -------------------------------------------------------------------------------------------------


%% Parameteres and initialization of NAO
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

%% UPDATING PARAMETERS
% ===========================================================
% Computation of the initial coefficients the polinomial trajectories for the controlled variables. 
% These coefficients are changed inmediately after the transition (really necessary when there is an impact)
gait_parameters_SS.transition = false; 
PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters_SS);
gait_parameters_SS.PolyCoeff = PolyCoeff;

% Computation of the initial coefficients the polinomial trajectories for the controlled variables. 
gait_parameters_DS.transition = false; 
PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters_DS);
gait_parameters_DS.PolyCoeff = PolyCoeff;

% Computation of the initial coefficients the polinomial trajectories for the controlled variables. 
gait_parameters_DS_final.transition = false; 
PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters_DS_final);
gait_parameters_DS_final.PolyCoeff = PolyCoeff;

% Computation of the initial coefficients the polinomial trajectories for the controlled variables.
gait_parameters_gait_SS.transition = false; 
PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters_gait_SS);
gait_parameters_gait_SS.PolyCoeff = PolyCoeff;

% Computation of the initial coefficients the polinomial trajectories for the controlled variables.
gait_parameters_SS_Stopping.transition = false; 
PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters_SS_Stopping);
gait_parameters_SS_Stopping.PolyCoeff = PolyCoeff;

% Computation of the initial coefficients the polinomial trajectories for the controlled variables. 
gait_parameters_DS_Stopping.transition = false; 
PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters_DS_Stopping);
gait_parameters_DS_Stopping.PolyCoeff = PolyCoeff;
%% OPTION - For plotting of the evolution of the CoM in "PEvents.m" file (while the solver is working on the dynamics)
% ===========================================================================================================
% % If we do not want to plot anything we can uncomment these 4 lines:
% global contC  
% global Stop   % 1-> If we want to pause the simulationn each time "PEvents" is executed
% contC = 1;    % It is used to create a vector that stores all the point of the CoM each time "PEvents" is executed
% Stop = 0;     % 1-> Each time  "PEvents" is executed the simulation will stop and it will wait until you press a key to continue
% % --------
% % NOTE If we want that the solver shows who many iterations have been performed we must define "DisplayIterNumber"
% global DisplayIterNumber  % To show how many iteration have been performed by the solver 
% DisplayIterNumber = 1;
global contA  % Necessary to store information each time "Objective function" is executed 
contA = 1;
% ====================================================================

%% Continue
%% Robot initial step DS and SS (Starting from REST)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Modelo esncial de la fase de inicio y del caminado
[t_gait,Xt_gait,t_SS,Xt_SS,t_DS,Xt_DS,t_DS_final,Xt_DS_final,gait_parameters_SS,gait_parameters_DS,gait_parameters_DS_final,gait_parameters_gait_SS] = ...
    robot_step_EssModel_Start_DS_SS_DS_gait_t(gait_parameters_SS,gait_parameters_DS,gait_parameters_DS_final,gait_parameters_gait_SS,Nstep);
% Condiciones iniciales para la fase de detención
X_final(1) = Xt_gait{1,1}(end,1) - S;
X_final(2) = Xt_gait{1,1}(end,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Modelo esencial de la fase de detención
[t_Stop,Xt_Stop,t_SS_Stop,Xt_SS_Stop,t_DS_Stop,Xt_DS_Stop,gait_parameters_SS_Stopping,gait_parameters_DS_Stopping,] = ...
    robot_step_EssModel_Stopping_SS_DS_t(X_final,gait_parameters_SS_Stopping,gait_parameters_DS_Stopping);%% Storing all the parameters and information of this simulation
% =====================================================================
samples_Starting_DS = length(t_DS);
samples_Starting_SS = length(t_SS);
samples_Starting_DS_final = length(t_DS_final);
samples_SS_gait = length(t_gait{1});
samples_SS_Stop = length(t_SS_Stop);
samples_DS_Stop = length(t_DS_Stop);
ZMPd_DS = zeros(1,samples_Starting_DS);
ZMPd_SS = zeros(1,samples_Starting_SS);
ZMPd_DS_final = zeros(1,samples_Starting_DS_final);
ZMPd_SS_gait = zeros(1,samples_SS_gait);
ZMPd_SS_Stop = zeros(1,samples_SS_Stop);
ZMPd_DS_Stop = zeros(1,samples_DS_Stop);
for i=1:samples_Starting_DS
        ZMPdX_DS(i) = polyval(gait_parameters_DS.ZMPxCoeff,t_DS(i));
        zt_DS(i) = polyval(gait_parameters_DS.PolyCoeff.hd1,t_DS(i));
        zpt_DS(i) = polyval(polyder(gait_parameters_DS.PolyCoeff.hd1),t_DS(i));
end 
for i=1:samples_Starting_SS
        ZMPdX_SS(i) = polyval(gait_parameters_SS.ZMPxCoeff,t_SS(i));
        zt_SS(i) = polyval(gait_parameters_SS.PolyCoeff.hd1,t_SS(i));
        zpt_SS(i) = polyval(polyder(gait_parameters_SS.PolyCoeff.hd1),t_SS(i));
end 
for i=1:samples_Starting_DS_final
        ZMPdX_DS_final(i) = polyval(gait_parameters_DS_final.ZMPxCoeff,t_DS_final(i));
        zt_DS_final(i) = polyval(gait_parameters_DS_final.PolyCoeff.hd1,t_DS_final(i));
        zpt_DS_final(i) = polyval(polyder(gait_parameters_DS_final.PolyCoeff.hd1),t_DS_final(i));
end 
for i=1:samples_SS_gait
        ZMPdX_gait_SS(i) = polyval(gait_parameters_gait_SS.ZMPxCoeff,t_gait{1}(i));
        zt_gait_SS(i) = polyval(gait_parameters_gait_SS.PolyCoeff.hd1,t_gait{1}(i));
        zpt_gait_SS(i) = polyval(polyder(gait_parameters_gait_SS.PolyCoeff.hd1),t_gait{1}(i));
end
ZMPd_gait = ZMPd_SS_gait;
for i=1:samples_SS_Stop
        ZMPdX_SS_Stop(i) = polyval(gait_parameters_SS_Stopping.ZMPxCoeff,t_SS_Stop(i));
        zt_SS_Stop(i) = polyval(gait_parameters_SS_Stopping.PolyCoeff.hd1,t_SS_Stop(i));
        zpt_SS_Stop(i) = polyval(polyder(gait_parameters_SS_Stopping.PolyCoeff.hd1),t_SS_Stop(i));
end
for i=1:samples_DS_Stop
        ZMPdX_DS_Stop(i) = polyval(gait_parameters_DS_Stopping.ZMPxCoeff,t_DS_Stop(i));
        zt_DS_Stop(i) = polyval(gait_parameters_DS_Stopping.PolyCoeff.hd1,t_DS_Stop(i));
        zpt_DS_Stop(i) = polyval(polyder(gait_parameters_DS_Stopping.PolyCoeff.hd1),t_DS_Stop(i));
end
ZMPd_Stop = [ZMPd_SS_Stop ZMPd_DS_Stop];
maxPosX = (Nstep+2)*S + max(Xt_gait{1,1}(:,1));
minPosX = min(Xt_DS(:,1)) - S;
%
if Animation && produceVideo
    writerobj = VideoWriter(strcat(['ReducedModel_',DataName],'.avi'));
    writerobj.FrameRate = framerate; % The smaller FrameRate the slower the video (Frames per second)
    open(writerobj);    
    disp('Making a video...')    
end
ZMPdS = cell(1,Nstep);
xsAll = [];
zsAll = [];
xpsAll = [];
tsAll = [];
xsAll_SS = [];
zsAll_SS = [];
xpsAll_SS = [];
tsAll_SS = [];
xs_gait_c = [];
zs_gait_c = [];
Xs_DS = sampling(Xt_DS',n); % Xs is the sampled evolution of the CoM  for step "j" in the LOCAL frame attached to the support foot
Xs_SS = sampling(Xt_SS',n);
Xs_DS_final = sampling(Xt_DS_final',n);
Xs_gait = sampling(Xt_gait{1}',n);
Xs_Stop = sampling(Xt_Stop',n);
ts_DS = sampling(t_DS',n);
ts_SS = sampling(t_SS',n);
ts_DS_final = sampling(t_DS_final',n);
ts_gait = sampling(t_gait{1}',n);
ts_stop = sampling(t_Stop',n);
zs_DS = sampling(zt_DS,n);
zs_SS = sampling(zt_SS,n);
zs_DS_final = sampling(zt_DS_final,n);
zs_SS_gait = sampling(zt_gait_SS,n);
zs_SS_Stop = sampling(zt_SS_Stop,n);
zs_DS_Stop = sampling(zt_DS_Stop,n);
%
ZMPdS_DS = sampling(ZMPd_DS,n);  % ZMPdS is the sampled ZMP for each step;
ZMPdS_SS = sampling(ZMPd_SS,n);
ZMPdS_DS_final = sampling(ZMPd_DS_final,n);
ZMPdS_gait = sampling(ZMPd_gait,n);
ZMPdS_Stop = sampling(ZMPd_Stop,n);
%
ZMPdsX_gait =[];
ZMPdsX_gait_c =[];
for i=1:Nstep
    nj = S*i;
    xs_gait = Xs_gait(1,:) + nj;
    ZMPdsX_gait = ZMPdS_gait(1,:) + nj;
    xs_gait_c = [xs_gait_c xs_gait];
    zs_gait_c = [zs_gait_c zs_SS_gait];
    ZMPdsX_gait_c = [ZMPdsX_gait_c ZMPdsX_gait];
end
xs_Stop = Xs_Stop(1,:) + (Nstep+1)*S;
ZMPds_Stop_x = ZMPdS_Stop(1,:) + (Nstep+1)*S;

xsAll = [Xs_DS(1,:) Xs_SS(1,:) Xs_DS_final(1,:) xs_gait_c xs_Stop];
zsAll = [zs_DS zs_SS zs_DS_final zs_gait_c zs_SS_Stop];
ZMPdsx_All = [ZMPdS_DS(1,:) ZMPdS_SS(1,:) ZMPdS_DS_final(1,:) ZMPdsX_gait_c ZMPds_Stop_x];
%
if Animation                
        % Sampled variables
        OnesRowVectS = ones(1,n);
        %
        xs = Xs_DS(1,:);
        xps = Xs_DS(2,:);                      % Sampled velocity in X of the CoM for step "j"
        zs = zs_DS; % Xs is the sampled evolution of the CoM  for step "j" in the LOCAL frame attached to the support foot        
        % zs = z0*OnesRowVectS;

        color = rand(1,3); % Random Color
        
        CoMIniEndPos = [xs(1),zs(1),xs(end),zs(end)];
        for i=1:n
            fig = figure (1);
            % if we want to draw all the trace of the pendulum we must put it "on"
            hold off
            plot([ZMPdS_DS(1,i),Xs_DS(1,i)],[0,zt_DS(i)],'-ko');
            hold on
            % The next part draw the initial and final position of the pendulum for each step
            % ------------------------------------------------------------------------------------
            plot([ZMPdS_DS(1,1),CoMIniEndPos(1,1)],[0,CoMIniEndPos(1,2)],'--o', 'color',[0.68,0.47,0]);
            % Final position
%             plot3([ZMPdS_DS(1,end),CoMIniEndPos(1,4)],[ZMPdS_DS(2,end),CoMIniEndPos(1,5)],[0,CoMIniEndPos(1,6)],'--o', 'color',[0.48,0.06,0.89]);
            % Trajectory of the ZMP                
            plot(ZMPdsx_All(1:i),0*(1:i),'LineWidth',LineWidth);
            % Trajectory of the CoM
            plot(xsAll(1:i),zsAll(1:i),'color',color,'LineWidth',LineWidth)
            axis([minPosX,maxPosX,0,0.2]); % axis([x0,xf,y0,yf,z0,zf]);
            xlabel('x [m]','interpreter','Latex','FontSize',FontSize);
            ylabel('z [m]','interpreter','Latex','FontSize',FontSize);
            title(['t = ' num2str(ts_DS(i)) ' [sec]']);
            grid on
            if  produceVideo
                writeVideo(writerobj,getframe(fig));
            end 
            pause(0.5)
        end
        xs = Xs_SS(1,:);
        xps = Xs_SS(2,:);                      % Sampled velocity in X of the CoM for step "j"
        CoMIniEndPos = [xs(1),zs(1),xs(end),zs(end)];
        for i=1:n
            fig = figure (1);
            % if we want to draw all the trace of the pendulum we must put it "on"
            hold off
            plot([ZMPdS_SS(1,i),Xs_SS(1,i)],[0,zt_SS(i)],'-ko');
            hold on
            % The next part draw the initial and final position of the pendulum for each step
            % ------------------------------------------------------------------------------------
            plot([ZMPdS_SS(1,1),CoMIniEndPos(1,1)],[0,CoMIniEndPos(1,2)],'--o', 'color',[0.68,0.47,0]);
            plot([ZMPdsx_All(1),xsAll(1)],[0,zsAll(1)],'--o', 'color',[0.68,0.47,0]);
            % Final position
%             plot3([ZMPdS_DS(1,end),CoMIniEndPos(1,4)],[ZMPdS_DS(2,end),CoMIniEndPos(1,5)],[0,CoMIniEndPos(1,6)],'--o', 'color',[0.48,0.06,0.89]);
            % Trajectory of the ZMP                
            plot(ZMPdsx_All(1:n),0*(1:n),'LineWidth',LineWidth);
            plot(ZMPdsx_All(n:n+i),0*(1:i+1),'LineWidth',LineWidth);
            % Trajectory of the CoM
            plot(xsAll(1:n),zsAll(1:n),'color',color,'LineWidth',LineWidth)
            plot(xsAll(n:(n+i)),zsAll(n:(n+i)),'color',color,'LineWidth',LineWidth)
            axis([minPosX,maxPosX,0,0.2]); % axis([x0,xf,y0,yf,z0,zf]);
            xlabel('x [m]','interpreter','Latex','FontSize',FontSize);
            ylabel('z [m]','interpreter','Latex','FontSize',FontSize);
            title(['t = ' num2str(ts_DS(end) + ts_SS(i)) ' [sec]']);
            grid on
            if  produceVideo
                writeVideo(writerobj,getframe(fig));
            end 
            pause(0.5)
        end
        xs = Xs_DS_final(1,:);
        xps = Xs_DS_final(2,:);                      % Sampled velocity in X of the CoM for step "j"; 
        CoMIniEndPos = [xs(1),zs(1),xs(end),zs(end)];
        for i=1:n
            fig = figure (1);
            % if we want to draw all the trace of the pendulum we must put it "on"
            hold off
            plot([ZMPdS_DS_final(1,i),Xs_DS_final(1,i)],[0,zt_DS_final(i)],'-ko');
            hold on
            % The next part draw the initial and final position of the pendulum for each step
            % ------------------------------------------------------------------------------------
%             plot3([ZMPdS_DS_final(1,1),CoMIniEndPos(1,1)],[ZMPdS_DS_final(2,1),CoMIniEndPos(1,2)],[0,CoMIniEndPos(1,3)],'--o', 'color',[0.68,0.47,0]);
            plot([ZMPdsx_All(1),xsAll(1)],[0,zsAll(1)],'--o', 'color',[0.68,0.47,0]);
            plot([ZMPdsx_All(n),xsAll(n)],[0,zsAll(n)],'--o', 'color',[0.68,0.47,0]);
            % Final position
%             plot3([ZMPdS_DS(1,end),CoMIniEndPos(1,4)],[ZMPdS_DS(2,end),CoMIniEndPos(1,5)],[0,CoMIniEndPos(1,6)],'--o', 'color',[0.48,0.06,0.89]);
            % Trajectory of the ZMP                
            plot(ZMPdsx_All(1:2*n),0*(1:2*n),'LineWidth',LineWidth);
            plot(ZMPdsx_All(2*n:2*n+i),0*(1:i+1),'LineWidth',LineWidth);
            % Trajectory of the CoM
            plot(xsAll(1:2*n),zsAll(1:2*n),'color',color,'LineWidth',LineWidth)
            plot(xsAll(2*n:(2*n+i)),zsAll(2*n:(2*n+i)),'color',color,'LineWidth',LineWidth)
            axis([minPosX,maxPosX,0,0.2]); % axis([x0,xf,y0,yf,z0,zf]);
            xlabel('x [m]','interpreter','Latex','FontSize',FontSize);
            ylabel('z [m]','interpreter','Latex','FontSize',FontSize);
            title(['t = ' num2str(ts_DS(end) + ts_SS(end) + ts_DS_final(i)) ' [sec]']);
            ts_start = ts_DS(end) + ts_SS(end) + ts_DS_final(end);
            grid on
            if  produceVideo
                writeVideo(writerobj,getframe(fig));
            end 
            pause(0.5)
        end
    for j=1:(Nstep + 1)
        xs = xsAll;
        zs = zsAll;
        aux = (3*n+1)+(j-1)*n;
        CoMIniEndPos = [xs(aux),zs(aux),xs(aux+n-1),zs(aux+n-1)];
        aux2 = 3*n+1;
        for i=1:n
            fig = figure (1);
            % if we want to draw all the trace of the pendulum we must put it "on"
            hold off
            plot([ZMPdsx_All(aux+i-1),xs(aux+i-1)],[0,zsAll(aux+i-1)],'-ko','HandleVisibility','off');
            hold on
            % The next part draw the initial and final position of the pendulum for each step
            % ------------------------------------------------------------------------------------
            plot([ZMPdsx_All(aux),CoMIniEndPos(1,1)],[0,CoMIniEndPos(1,2)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
            plot([ZMPdsx_All(1),xsAll(1)],[0,zsAll(1)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
            plot([ZMPdsx_All(n),xsAll(n)],[0,zsAll(n)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
            for k=2:(j+1)
                plot([ZMPdsx_All((k+1)*n),xsAll((k+1)*n)],[0,zsAll((k+1)*n)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
            end
            for m=1:j
                aux3 = (3*n+1)+(m-1)*n;
                plot(ZMPdsx_All(aux3:aux3+n-1),0*(aux3:aux3+n-1),'LineWidth',LineWidth,'HandleVisibility','off','HandleVisibility','off');
            end
            % Final position
%             plot3([ZMPdS_DS(1,end),CoMIniEndPos(1,4)],[ZMPdS_DS(2,end),CoMIniEndPos(1,5)],[0,CoMIniEndPos(1,6)],'--o', 'color',[0.48,0.06,0.89]);
            % Trajectory of the ZMP
            plot(ZMPdsx_All(1:aux2-1),0*(1:aux2-1),'LineWidth',LineWidth,'HandleVisibility','off','HandleVisibility','off');
            plot(ZMPdsx_All(aux:(aux+i-1)),0*(1:i+1-1),'LineWidth',LineWidth,'HandleVisibility','off');
            % Trajectory of the CoM
            plot(xsAll(1:aux),zsAll(1:aux),'color',color,'LineWidth',LineWidth,'HandleVisibility','off')
            plot(xsAll(aux:(aux+i-1)),zsAll(aux:(aux+i-1)),'color',color,'LineWidth',LineWidth,'HandleVisibility','off')
            if i==8 && j==(Nstep+1)
                plot([ZMPdsx_All(end),xsAll(end)],[0,zsAll(end)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
            end
            axis([minPosX,maxPosX,0,0.2]); % axis([x0,xf,y0,yf,z0,zf]);
            xlabel('x [m]','interpreter','Latex','FontSize',FontSize);
            ylabel('z [m]','interpreter','Latex','FontSize',FontSize);
            if j==(Nstep+1)
                title(['t = ' num2str(ts_start + Nstep*ts_gait(end) + ts_stop(i)) ' [sec]']);
            else
                title(['t = ' num2str(ts_start + ts_gait(end)*(j-1) + ts_gait(i)) ' [sec]']);
            end
            grid on
            if  produceVideo
                writeVideo(writerobj,getframe(fig));
            end 
            pause(0.5)
        end
    end
        xs = Xs_Stop(1,:);
        xps = Xs_Stop(2,:);                      % Sampled velocity in X of the CoM for step "j"
        CoMIniEndPos = [xs(1),zs(1),xs(end),zs(end)];
%     for i=1:n
%             fig = figure (1);
%             aux2 = (3+Nstep)*n;
%             % if we want to draw all the trace of the pendulum we must put it "on"
%             hold off
%             plot3([ZMPdsx_All(aux2+i),xsAll(aux2+i)],[ZMPdsy_All(aux2+i),ysAll(aux2+i)],[0,zsAll(aux2+i)],'-ko','HandleVisibility','off');
%             hold on
%             % The next part draw the initial and final position of the pendulum for each step
%             % ------------------------------------------------------------------------------------
%             plot3([ZMPdsx_All((j+3)*n),xsAll((j+3)*n)],[ZMPdsy_All((j+3)*n),ysAll((j+3)*n)],[0,zsAll((j+3)*n)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
%             plot3([ZMPdsx_All(1),xsAll(1)],[ZMPdsy_All(1),ysAll(1)],[0,zsAll(1)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
%             plot3([ZMPdsx_All(n),xsAll(n)],[ZMPdsy_All(n),ysAll(n)],[0,zsAll(n)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
%             for k=2:(j+1)
%                 plot3([ZMPdsx_All((k+1)*n),xsAll((k+1)*n)],[ZMPdsy_All((k+1)*n),ysAll((k+1)*n)],[0,zsAll((k+1)*n)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
%             end
%             % Final position
% %             plot3([ZMPdS_DS(1,end),CoMIniEndPos(1,4)],[ZMPdS_DS(2,end),CoMIniEndPos(1,5)],[0,CoMIniEndPos(1,6)],'--o', 'color',[0.48,0.06,0.89]);
%             % Trajectory of the ZMP                
%             plot3(ZMPdsx_All(1:aux2),ZMPdsy_All(1:aux2),0*(1:aux2),'LineWidth',LineWidth,'HandleVisibility','off');
%             plot3(ZMPdsx_All(aux2:aux2+i),ZMPdsy_All(aux2:aux2+i),0*(1:i+1),'LineWidth',LineWidth,'HandleVisibility','off');
%             % Trajectory of the CoM
%             plot3(xsAll(1:aux2),ysAll(1:aux2),zsAll(1:aux2),'color',color,'LineWidth',LineWidth,'HandleVisibility','off')
%             plot3(xsAll(aux2:aux2+i),ysAll(aux2:aux2+i),zsAll(aux2:aux2+i),'color',color,'LineWidth',LineWidth,'HandleVisibility','off')
%             if i==8
%                 plot3([ZMPdsx_All(end),xsAll(end)],[ZMPdsy_All(end),ysAll(end)],[0,zsAll(end)],'--o', 'color',[0.68,0.47,0],'HandleVisibility','off');
%             end
%             axis([minPosX,maxPosX,minPosY,maxPosY,0,1]); % axis([x0,xf,y0,yf,z0,zf]);
%             xlabel('x [m]','interpreter','Latex','FontSize',FontSize);
%             ylabel('y [m]','interpreter','Latex','FontSize',FontSize);
%             zlabel('z [m]','interpreter','Latex','FontSize',FontSize);
%             title(['t = ' num2str(ts_start + Nstep*ts_gait(end) + ts_stop(i)) ' [sec]']);
%             grid on
%             if  produceVideo
%                 writeVideo(writerobj,getframe(fig));
%             end 
%             pause(0.5)
%     end
end

    % ===============================================================
if Animation && produceVideo
    close(writerobj);
    disp('Video produced successfully')
    disp('------------------------------')
end
Solution.t_gait = t_gait;
Solution.Xt_gait = Xt_gait;
Solution.t_SS = t_SS;
Solution.Xt_SS = Xt_SS;
Solution.t_DS = t_DS;
Solution.Xt_DS = Xt_DS;
Solution.t_DS_final = t_DS_final;
Solution.Xt_DS_final = Xt_DS_final;
Solution.t_Stop = t_Stop;
Solution.Xt_Stop = Xt_Stop;
Solution.t_SS_Stop = t_SS_Stop;
Solution.Xt_SS_Stop = Xt_SS_Stop;
Solution.t_DS_Stop = t_DS_Stop;
Solution.Xt_DS_Stop = Xt_DS_Stop;

% Build just ONE structure with all the information
InfBiped.robot = robot;
% % InfROMEO.iniParameters = initParam;
InfBiped.gait_parameters_SS = gait_parameters_SS;
InfBiped.gait_parameters_DS = gait_parameters_DS;
InfBiped.gait_parameters_DS_final = gait_parameters_DS_final;
InfBiped.gait_parameters_gait_SS = gait_parameters_gait_SS;
InfBiped.gait_parameters_SS_Stopping = gait_parameters_SS_Stopping;
InfBiped.gait_parameters_DS_Stopping = gait_parameters_DS_Stopping;
InfBiped.Solution = Solution;
disp(['Saving data as: ' DataName]);
disp('------------------------------');
save(DataName,'InfBiped')

%% End of the code
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------