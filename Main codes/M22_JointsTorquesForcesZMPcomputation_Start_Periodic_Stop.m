close all;
clear all;
clc
% ---------------------------------------------------------------------------------------------
currentfolder = pwd; % save current path
cd ..                % go one folder down (go out of the current folder and stay in the previous one)
add_paths;           % add all folders where all files are founded in order to acces to them
cd(currentfolder);   % return to the original path
% ---------------------------------------------------------------------------------------------

echo on
% Computing of joint trajectories positions velocities and accelerations  based on the CoM trajectory (Position and velocity)
% Also, computing reaction moment and force on the support foot, joint torques and ZMP.
% ==============================================================================================
% Creation: 05/03/2021
% Last modification: -/-/-
% --------------------------------------------------------------------------------------------------
% This file load the CoM trajectory, gait information and others parameters to compute the Reaction
% moment and force of the ground, joint torques and ZMP produced.
% IN HERE the evolution of the ZMP COULD be OUTSIDE of the sole, all deppends of the CoM trajectory in position and
% velocity and the motion given to the free foot.
% --------------------------------------------------------------------------------------------------
%
%
echo off
% -----------------------------------------------
Hour = datestr(now,13);
disp([Hour ' -> Reading data']);
% -----------------------------------------------
% Options
% -----------------------------------------------
DataName = 'InfBiped_Param01_Start_Gait_SS_Stop'; % File produced in "M04"

anim = 1; % Do you want animation? 1-> yes, 0-> no
% -------------------------------------------------------------------------------------------------
% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
global OptionContVar  % Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn" and "hppd_Polyn".
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t".
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x".
OptionContVar = 1;
% -----------------------------------------------

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

% input
% -----------------------------------------------
load(DataName);
NameAnim = ['anim_', DataName];
% -----------------------------------------------
robot = InfBiped.robot;
gait_parameters_SS = InfBiped.gait_parameters_SS;
gait_parameters_DS = InfBiped.gait_parameters_DS;
gait_parameters_DS_final = InfBiped.gait_parameters_DS_final;
gait_parameters_gait = InfBiped.gait_parameters_gait_SS;
gait_parameters_SS_Stopping = InfBiped.gait_parameters_SS_Stopping;
gait_parameters_DS_Stopping = InfBiped.gait_parameters_DS_Stopping;
Solution = InfBiped.Solution;
S_SS = gait_parameters_SS.S; %Step_length/2;

t_gait = Solution.t_gait;
Xt_gait = Solution.Xt_gait;
t_SS = Solution.t_SS;    
Xt_SS = Solution.Xt_SS;  
t_DS = Solution.t_DS;    
Xt_DS = Solution.Xt_DS; 
t_DS_final = Solution.t_DS_final;
Xt_DS_final = Solution.Xt_DS_final ;
t_Stop = Solution.t_Stop;
Xt_Stop = Solution.Xt_Stop;
t_SS_Stop = Solution.t_SS_Stop;
Xt_SS_Stop = Solution.Xt_SS_Stop;
t_DS_Stop = Solution.t_DS_Stop;
Xt_DS_Stop = Solution.Xt_DS_Stop;

Nstep = 3; % number of steps

qt = cell(1,Nstep);
qDt = cell(1,Nstep);
qDDt = cell(1,Nstep);
ZMP = cell(1,Nstep);
F =cell(1,Nstep);
M = cell(1,Nstep);
Tau = cell(1,Nstep);

qt_Starting = cell(1,1);
qDt_Starting = cell(1,1);
qDDt_Starting = cell(1,1);
ZMP_Starting = cell(1,1);
F_Starting = cell(1,1);
M_Starting = cell(1,1);
Tau_Starting = cell(1,1);

qt_SS = cell(1,Nstep);
qDt_SS = cell(1,Nstep);
qDDt_SS = cell(1,Nstep);
ZMP_SS = cell(1,Nstep);
F_SS =cell(1,Nstep);
M_SS = cell(1,Nstep);
Tau_SS = cell(1,Nstep);

fprintf('DS Starting Phase');
DateVecOld = datevec(Hour);
Hour = datestr(now,13);
disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
disp([Hour ' -> Computing joint positions velocities and acceleration of the DS Starting phase based on the CoM position and velocity']);
[qt_DS_Starting,qDt_DS_Starting,qDDt_DS_Starting] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_DS,gait_parameters_DS,t_DS);
%
fprintf('SS Starting Phase');
DateVecOld = datevec(Hour);
Hour = datestr(now,13);
disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
disp([Hour ' -> Computing joint positions velocities and acceleration of the DS Starting phase based on the CoM position and velocity']);
[qt_SS_Starting,qDt_SS_Starting,qDDt_SS_Starting] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_SS,gait_parameters_SS,t_SS);
%
fprintf('DS_final Starting Phase');
DateVecOld = datevec(Hour);
Hour = datestr(now,13);
disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
disp([Hour ' -> Computing joint positions velocities and acceleration of the DS_final Starting phase based on the CoM position and velocity']);
[qt_DS_f_Starting,qDt_DS_f_Starting,qDDt_DS_f_Starting] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_DS_final,gait_parameters_DS_final,t_DS_final);
%
Xt_Starting = [Xt_DS; Xt_SS; Xt_DS_final];
t_Starting = [t_DS; t_DS(end) + t_SS; t_DS(end) + t_SS(end) + t_DS_final];
qt_Starting = [qt_DS_Starting,qt_SS_Starting,qt_DS_f_Starting];
samples_SS_Starting = size(t_SS,1);
samples_DS_Starting = size(t_DS,1);
samples_DS_f_Starting = size(t_DS_final,1);
samples_Starting = size(t_SS,1) + size(t_DS,1) + size(t_DS_final,1);
% % Torque, force and ZMP computation
    % -----------------------------------------------
    DateVecOld = datevec(Hour);
    Hour = datestr(now,13);
    disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
    disp([Hour ' -> Computation of global reaction mmoment and force, ZMP and torques based on "q", "qp" and "qpp" in DS Starting phase ...']);
    % ------------------------------------------------
%
% ------------------------------------------------
ZMP_Starting = zeros(2,samples_Starting);
F_Starting = zeros(3,samples_Starting);
M_Starting = zeros(3,samples_Starting);
Tau_Starting = zeros(robot.joints,samples_Starting);
    
ZMP_DS_Starting = zeros(2,samples_DS_Starting);
F_DS_Starting = zeros(3,samples_DS_Starting);
M_DS_Starting = zeros(3,samples_DS_Starting);
Tau_DS_Starting = zeros(robot.joints,samples_DS_Starting);

ZMP_SS_Starting = zeros(2,samples_SS_Starting);
F_SS_Starting = zeros(3,samples_SS_Starting);
M_SS_Starting = zeros(3,samples_SS_Starting);
Tau_SS_Starting = zeros(robot.joints,samples_SS_Starting);
    
ZMP_DS_f_Starting = zeros(2,samples_DS_f_Starting);
F_DS_f_Starting = zeros(3,samples_DS_f_Starting);
M_DS_f_Starting = zeros(3,samples_DS_f_Starting);
Tau_DS_f_Starting = zeros(robot.joints,samples_DS_f_Starting);    
for i = 1:samples_DS_Starting
        % "Newton_Euler.m" Algorithm
        [F_DS_Starting(:,i ),M_DS_Starting(:,i ),Tau_DS_Starting(:,i )]= Newton_Euler(qt_DS_Starting(:,i),qDt_DS_Starting(:,i),qDDt_DS_Starting(:,i));
        ZMP_DS_Starting(1,i) = -M_DS_Starting(2,i)/F_DS_Starting(3,i);
        ZMP_DS_Starting(2,i) =  M_DS_Starting(1,i)/F_DS_Starting(3,i);
%         M_SS{j}(3,i)
end
for i = 1:samples_SS_Starting
        % "Newton_Euler.m" Algorithm
        [F_SS_Starting(:,i ),M_SS_Starting(:,i ),Tau_SS_Starting(:,i )]= Newton_Euler(qt_SS_Starting(:,i),qDt_SS_Starting(:,i),qDDt_SS_Starting(:,i));
        ZMP_SS_Starting(1,i) = -M_SS_Starting(2,i)/F_SS_Starting(3,i);
        ZMP_SS_Starting(2,i) =  M_SS_Starting(1,i)/F_SS_Starting(3,i);
%         M_SS{j}(3,i)
end
for i = 1:samples_DS_f_Starting
        % "Newton_Euler.m" Algorithm
        [F_DS_f_Starting(:,i ),M_DS_f_Starting(:,i ),Tau_DS_f_Starting(:,i )]= Newton_Euler(qt_DS_f_Starting(:,i),qDt_DS_f_Starting(:,i),qDDt_DS_f_Starting(:,i));
        ZMP_DS_f_Starting(1,i) = -M_DS_f_Starting(2,i)/F_DS_f_Starting(3,i);
        ZMP_DS_f_Starting(2,i) =  M_DS_f_Starting(1,i)/F_DS_f_Starting(3,i);
%         M_SS{j}(3,i)
end
%
F_Starting = [F_DS_Starting,F_SS_Starting,F_DS_f_Starting];
M_Starting = [M_DS_Starting,M_SS_Starting,M_DS_f_Starting];
ZMP_Starting = [ZMP_DS_Starting,ZMP_SS_Starting,ZMP_DS_f_Starting];
%
samplesAct = 0;
for j=1:Nstep
    X0 = Xt_gait{j}(1,:); % Initial condition of the CoM
    % % Torque, force and ZMP computation
    % -----------------------------------------------
    fprintf('Step %d out of %d \n',j,Nstep);
    DateVecOld = datevec(Hour);
    Hour = datestr(now,13);
    disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
    disp([Hour ' -> Computing joint positions velocities and acceleration of the SS phase based on the CoM position and velocity']);
    [qt_SS{j},qDt_SS{j},qDDt_SS{j}] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_gait{j},gait_parameters_gait,t_gait{j});
    % ------------------------------------------------
    qt{j} = qt_SS{j};
    qDt{j} = qDt_SS{j};
    qDDt{j} = qDDt_SS{j};
    samples = size(t_gait{j},1);
    
    % % Torque, force and ZMP computation
    % -----------------------------------------------
    DateVecOld = datevec(Hour);
    Hour = datestr(now,13);
    disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
    disp([Hour ' -> Computation of global reaction mmoment and force, ZMP and torques based on "q", "qp" and "qpp" in SS phase ...']);
    % ------------------------------------------------
    ZMP{j} = zeros(2,samples);
    F{j} = zeros(3,samples);
    M{j} = zeros(3,samples);
    Tau{j} = zeros(robot.joints,samples);
    
    for i = 1:samples
        % "Newton_Euler.m" Algorithm
        [F{j}(:,i ),M{j}(:,i ),Tau{j}(:,i )]= Newton_Euler(qt{j}(:,i),qDt{j}(:,i),qDDt{j}(:,i));
        ZMP{j}(1,i) = -M{j}(2,i)/F{j}(3,i);
        ZMP{j}(2,i) =  M{j}(1,i)/F{j}(3,i);
%         M_SS{j}(3,i)
    end
    
    % -----------------------------------------------
    DateVecOld = datevec(Hour);
    Hour = datestr(now,13);
    disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
    disp([Hour ' -> Computation of global reaction mmoment and force and ZMP based on "q", "qp" and "qpp" in DS phase ...']);
    % ------------------------------------------------
 
    % Building the joint trajectories for all the steps
    if mod(j,2) % If j is impair
        qtAll(:,1+samplesAct:samples+samplesAct) = qt{j};  % joint position q(t) for all the steps
        qDtAll(:,1+samplesAct:samples+samplesAct) = qDt{j};  % joint velocities q(t) for all the steps
        qDDtAll(:,1+samplesAct:samples+samplesAct) = qDDt{j};  % joint accelerations q(t) for all the steps
    else
        % Swaping according to the impact (see impact_Pos_Vel.m)
        %      qtSwap = [qt{j}(12,:); -qt{j}([11,10,9],:); qt{j}([8,7,6,5],:); -qt{j}([4,3,2],:); qt{j}(1,:); -qt{j}(13,:);  qt{j}(14:31,:)];
        %     qDtSwap = [qDt{j}(12,:); -qDt{j}([11,10,9],:); qDt{j}([8,7,6,5],:); -qDt{j}([4,3,2],:); qDt{j}(1,:); -qDt{j}(13,:);  qDt{j}(14:31,:)];
        %     qDDtSwap = [qDDt{j}(12,:); -qDDt{j}([11,10,9],:); qDDt{j}([8,7,6,5],:); -qDDt{j}([4,3,2],:); qDDt{j}(1,:); -qDDt{j}(13,:);  qDDt{j}(14:31,:)];
        qtSwap = -qt{j}([6,5,4,3,2,1],:);
        qDtSwap = -qDt{j}([6,5,4,3,2,1],:);
        qDDtSwap = -qDDt{j}([6,5,4,3,2,1],:);
        qtAll(:,1+samplesAct:samples+samplesAct) = qtSwap;  % joint position q(t) for all the steps
        qDtAll(:,1+samplesAct:samples+samplesAct) = qDtSwap;  % joint velocities q(t) for all the steps
        qDDtAll(:,1+samplesAct:samples+samplesAct) = qDDtSwap;  % joint accelerations q(t) for all the steps
    end
    F_All(:,1+samplesAct:samples+samplesAct) = F{j};
    M_All(:,1+samplesAct:samples+samplesAct) = M{j};
    Tau_All(:,1+samplesAct:samples+samplesAct) = Tau{j};
    ZMP_All(:,1+samplesAct:samples+samplesAct) = ZMP{j};
    samplesAct = samplesAct + samples;
    % ---------------------------------------------------------
end
fprintf('SS Stopping Phase');
DateVecOld = datevec(Hour);
Hour = datestr(now,13);
disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
disp([Hour ' -> Computing joint positions velocities and acceleration of the SS Stopping phase based on the CoM position and velocity']);
[qt_SS_Stopping,qDt_SS_Stopping,qDDt_SS_Stopping] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_SS_Stop,gait_parameters_SS_Stopping,t_SS_Stop);
%
fprintf('DS Stopping Phase');
DateVecOld = datevec(Hour);
Hour = datestr(now,13);
disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
disp([Hour ' -> Computing joint positions velocities and acceleration of the SS Stopping phase based on the CoM position and velocity']);
[qt_DS_Stopping,qDt_DS_Stopping,qDDt_DS_Stopping] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_DS_Stop,gait_parameters_DS_Stopping,t_DS_Stop);
%
qt_Stopping = [qt_SS_Stopping,qt_DS_Stopping];
%% Walking ANIMATION
% ==============================================================================
if anim
    dataS = cell(Nstep+1,1); % Sampled joint positions
    disp('Animation...')
    % This part is just to make the animation faster, the larger "n" the slower and finer the animation
    n = 10; % Number of samples of vector
    dataS{1,1} = sampling(qt_Starting,n);
    for i=2:(Nstep+1)
        qS = sampling(qt{i-1},n); % sampling of joint position "q"
        % We don't need the joint velocities to draw the robot, so we don't sample the velocity "qp"
        dataS{i,1} = qS;
    end
    dataS{Nstep+2,1} = sampling(qt_Stopping,n);
    framerate = 5;
    animation(dataS,Nstep+2,NameAnim,framerate); % ("parametro"= Numero de pasos a observar al final de la simulaci�n y a grabar en el video
    animation_stick(dataS,Nstep);    %"parametro"= Numero de pasos que se dibujar�n
    disp(['Animation stored as: ' NameAnim]);
    disp('----------------------------------');
end
% ==============================================================================
%% PLOTS
% =======================================================
DateVecOld = datevec(Hour);
Hour = datestr(now,13);
disp(['Elapsed time: ' datestr(etime(datevec(Hour),DateVecOld)/86400, 'HH:MM:SS')]);
disp([Hour ' -> Plotting...']);
% -----------------------------------------
% In the variable "plots" we chose which plot we want to show...
plots(1) = 1;
plots(2) = 1;
plots(3) = 1;
plots(4) = 1;
plots(5) = 0;
plots(6) = 0;
plots(7) = 0;
plots(8) = 0;
% plots(1) -> joint position, velocity and acceleration
% plots(2) -> reaction force and moment
% plots(3) -> joint torques
% plots(4) -> ZMP time
% plots(5) -> ZMP foot
% plots(6) -> CoM time
% plots(7) -> CoM X-Y
% plots(8) -> CoM samples
% ------------------------------------------------------------
% General file for plotting:
colors = 'krbgymc'; % Posible color for the lines of each step. Maximum 7 steps (since there are 7 different colors)
for j=1:Nstep
    hold on
    LineType = colors(j);
    graphix(plots,Xt_gait{j},Tau{j},t_gait{j},qt{j},qDt{j},qDDt{j},F{j},M{j},ZMP{j},LineType,5);
end
% ------------------------------------------------------------
%% End of the code

% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------