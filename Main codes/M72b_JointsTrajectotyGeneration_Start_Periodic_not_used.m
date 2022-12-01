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
% DataName = 'Param01_StartingPhase_WalkingPhase_SS_DS_repeat_nsteps_10ms';
DataName = 'Param03_StartingPhase_WalkingPhase_only_SS_nsteps';
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
% % %
Biped_param_DS = SS_bipedo_StartingDS();
Biped_param_SS = SS_bipedo_StartingSS();
Biped_param_DS_final = SS_bipedo_StartingDS_final();
Biped_param_gait_SS = SSParamComRob_ZMPx_var();
% ---------------------------------------------------------
Nstep=7;
global gait_parameters_SS gait_parameters_DS gait_parameters_DS_final gait_parameters_gait_SS 
gait_parameters_SS = Biped_param_SS.gait_parameters;
gait_parameters_DS = Biped_param_DS.gait_parameters;
gait_parameters_DS_final = Biped_param_DS_final.gait_parameters;
gait_parameters_gait_SS = Biped_param_gait_SS.gait_parameters;
%
S = Biped_param_gait_SS.gait_parameters.S;
% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
global OptionContVar  % Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_t" and "hppd_Polyn_t". 
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t". 
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x". 
OptionContVar = 1;
% -------------------------------------------------------------------------------------------------


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
[t_gait,Xt_gait,t_SS,Xt_SS,t_DS,Xt_DS,t_DS_final,Xt_DS_final,gait_parameters_SS,gait_parameters_DS,gait_parameters_DS_final,gait_parameters_gait_SS] = ...
    robot_step_EssModel_Start_DS_SS_DS_gait_t(gait_parameters_SS,gait_parameters_DS,gait_parameters_DS_final,gait_parameters_gait_SS,1);

[qt_DS,qDt_DS,qDDt_DS] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_DS,gait_parameters_DS,t_DS);
% robot = robot_move(robot,qt_DS(:,end));
% robot.qD = qDt_DS(:,end);
%
[qt_SS,qDt_SS,qDDt_SS] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_SS,gait_parameters_SS,t_SS);
% robot = robot_move(robot,qt_SS(:,end));
% robot.qD = qDt_SS(:,end);
%
[qt_DS_final,qDt_DS_final,qDDt_DS_final] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_DS_final,gait_parameters_DS_final,t_DS_final);
% robot = robot_move(robot,qt_DS_final(:,end));
% robot.qD = qDt_DS_final(:,end);
%
[qt_gait_SS1,qDt_gait_SS1,qDDt_gait_SS1] = joints_fromCoMpos_Vel_HZDtime(robot,Xt_gait{1,1},gait_parameters_gait_SS,t_gait{1,1});
qt_gait_SS1_orig = qt_gait_SS1; % Esto se agregó posteriormente para el programa M80 de comparación
qDt_gait_SS1_orig = qDt_gait_SS1; % Esto se agregó posteriormente para el programa M80 de comparación
qDDt_gait_SS1_orig = qDDt_gait_SS1; % Esto se agregó posteriormente para el programa M80 de comparación
% Relabeling de posiciones debido al cambio de soporte
qt_gait_SS1(1:6,:) = [-qt_gait_SS1(6,:);-qt_gait_SS1(5,:);-qt_gait_SS1(4,:);-qt_gait_SS1(3,:);-qt_gait_SS1(2,:);-qt_gait_SS1(1,:)];
% Relabeling de velocidades debido al cambio de soporte
qDt_gait_SS1(1:6,:) = [-qDt_gait_SS1(6,:);-qDt_gait_SS1(5,:);-qDt_gait_SS1(4,:);-qDt_gait_SS1(3,:);-qDt_gait_SS1(2,:);-qDt_gait_SS1(1,:)];
% Relabeling de aceleraciones debido al cambio de soporte
qDDt_gait_SS1(1:6,:) = [-qDDt_gait_SS1(6,:);-qDDt_gait_SS1(5,:);-qDDt_gait_SS1(4,:);-qDDt_gait_SS1(3,:);-qDDt_gait_SS1(2,:);-qDDt_gait_SS1(1,:)];
%
qt_all_SS_DS = cell(1,Nstep);
qDt_all_SS_DS = cell(1,Nstep);
qDDt_all_SS_DS = cell(1,Nstep);
%
qt_all_SS_DS_orig = cell(1,Nstep);
qDt_all_SS_DS_orig = cell(1,Nstep);
qDDt_all_SS_DS_orig = cell(1,Nstep);
%
t_all_SS_DS = cell(1,Nstep);
for j=1:Nstep
    if j>1
        if mod(j,2)==0 % Si es par
            qt_all_SS_DS{j} = [qt_all_SS_DS{j-1},qt_gait_SS1_orig];
            qDt_all_SS_DS{j} = [qDt_all_SS_DS{j-1},qDt_gait_SS1_orig];
            qDDt_all_SS_DS{j} = [qDDt_all_SS_DS{j-1},qDDt_gait_SS1_orig];
        else
            qt_all_SS_DS{j} = [qt_all_SS_DS{j-1},qt_gait_SS1];
            qDt_all_SS_DS{j} = [qDt_all_SS_DS{j-1},qDt_gait_SS1];
            qDDt_all_SS_DS{j} = [qDDt_all_SS_DS{j-1},qDDt_gait_SS1];
        end
    qt_all_SS_DS_orig{j} = [qt_all_SS_DS_orig{j-1},qt_gait_SS1_orig];
    qDt_all_SS_DS_orig{j} = [qDt_all_SS_DS_orig{j-1},qDt_gait_SS1_orig];
    qDDt_all_SS_DS_orig{j} = [qDDt_all_SS_DS_orig{j-1},qDDt_gait_SS1_orig]; 
    %
    t_all_SS_DS{j} = [t_all_SS_DS{j-1};t_all_SS_DS{j-1}(end)+ t_gait{1}];
    else
        qt_all_SS_DS{j} = qt_gait_SS1;
        qDt_all_SS_DS{j} = qDt_gait_SS1;
        qDDt_all_SS_DS{j} = qDDt_gait_SS1;
        %
        qt_all_SS_DS_orig{j} = qt_gait_SS1_orig;
        qDt_all_SS_DS_orig{j} = qDt_gait_SS1_orig;
        qDDt_all_SS_DS_orig{j} = qDDt_gait_SS1_orig;
        t_all_SS_DS{j} = t_gait{1};
    end
end
%
q_all = [qt_DS qt_SS qt_DS_final qt_all_SS_DS{Nstep}];
qD_all = [qDt_DS qDt_SS qDt_DS_final qDt_all_SS_DS{Nstep}];
qDD_all = [qDDt_DS qDDt_SS qDDt_DS_final qDDt_all_SS_DS{Nstep}];
t_all = [t_DS;
        t_DS(end) + t_SS;
        t_DS(end) + t_SS(end) + t_DS_final;
        t_DS(end) + t_SS(end) + t_DS_final(end) + t_all_SS_DS{Nstep}];
%
q_all_orig = [qt_DS qt_SS qt_DS_final qt_all_SS_DS_orig{Nstep}];
qD_all_orig = [qDt_DS qDt_SS qDt_DS_final qDt_all_SS_DS_orig{Nstep}];
qDD_all_orig = [qDDt_DS qDDt_SS qDDt_DS_final qDDt_all_SS_DS_orig{Nstep}];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Graficas de las posiciones
figure(3)
title('Posiciones')
subplot(2,3,1)
% plot(t_all,q_all(1,:));
plot(t_all,q_all(1,:)); %Trayectoria introducida al robot
title('q1');
%
subplot(2,3,2)
% plot(t_all,q_all(2,:));
plot(t_all,q_all(2,:)); %Trayectoria introducida al robot
title('q2');
%
subplot(2,3,3)
% plot(t_all,q_all(3,:));
plot(t_all,q_all(3,:)); %Trayectoria introducida al robot
title('q3');
%
subplot(2,3,4)
% plot(t_all,q_all(4,:));
plot(t_all,q_all(4,:)); %Trayectoria introducida al robot
title('q4');
%
subplot(2,3,5)
% plot(t_all,q_all(5,:));
plot(t_all,q_all(5,:)); %Trayectoria introducida al robot
title('q5');
%
subplot(2,3,6)
% plot(t_all,q_all(6,:));
plot(t_all,q_all(6,:)); %Trayectoria introducida al robot
title('q6');
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Gráficas de las velocidades
figure(5)
title('Velocidades')
subplot(2,3,1)
% plot(t_all,qD_all(1,:));
plot(t_all,qD_all(1,:)); %Trayectoria introducida al robot
title('qp1');
%
subplot(2,3,2)
% plot(t_all,qD_all(2,:));
plot(t_all,qD_all(2,:)); %Trayectoria introducida al robot
title('qp2');
%
subplot(2,3,3)
% plot(t_all,qD_all(3,:));
plot(t_all,qD_all(3,:)); %Trayectoria introducida al robot
title('qp3');
%
subplot(2,3,4)
% plot(t_all,qD_all(4,:));
plot(t_all,qD_all(4,:)); %Trayectoria introducida al robot
title('qp4');
%
subplot(2,3,5)
% plot(t_all,qD_all(5,:));
plot(t_all,qD_all(5,:)); %Trayectoria introducida al robot
title('qp5');
%
subplot(2,3,6)
% plot(t_all,qD_all(6,:));
plot(t_all,qD_all(6,:)); %Trayectoria introducida al robot
title('qp6');
%
%//////////////////////////////////////////////////////////////////////////
Solution.q_all = q_all;
Solution.qD_all = qD_all;
Solution.qDD_all = qDD_all;
Solution.t_all = t_all;
Solution.q_orig = q_all_orig;
Solution.qD_orig = qD_all_orig;
Solution.qDD_orig = qDD_all_orig;
Solution.qDD_all = qDD_all;
Solution.t_end_DS_final = t_DS(end) + t_SS(end) + t_DS_final(end);
Solution.t_end_DS1 = t_DS(end) + t_SS(end) + t_DS_final(end) + t_gait{1,1}(end);
Solution.t_SS = t_SS;
Solution.Xt_SS = Xt_SS;
Solution.t_DS = t_DS;
Solution.Xt_DS = Xt_DS;
Solution.t_DS_final = t_DS_final;
Solution.Xt_DS_final = Xt_DS_final;
Solution.t_gait = t_gait{1};
Solution.Xt_gait = Xt_gait;
Solution.qt_DS = qt_DS;
Solution.qDt_DS = qDt_DS;
Solution.qDDt_DS = qDDt_DS;
Solution.qt_SS = qt_SS;
Solution.qDt_SS = qDt_SS;
Solution.qDDt_SS = qDDt_SS;
Solution.qt_DS_final = qt_DS_final;
Solution.qDt_DS_final = qDt_DS_final;
Solution.qDDt_DS_final = qDDt_DS_final;
Solution.qt_all_SS_DS = qt_all_SS_DS_orig{Nstep};
Solution.qDt_all_SS_DS = qDt_all_SS_DS_orig{Nstep};
Solution.qDDt_all_SS_DS = qDDt_all_SS_DS_orig{Nstep};
Solution.S = S;
Solution.Nstep = Nstep;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
InfBiped.Solution = Solution;
disp(['Saving data as: ' DataName]);
disp('------------------------------');
save(DataName,'InfBiped')
%Creación de los archivos .txt para llevarlos a pyhton
%% End of the code
toc; % Finishing of counting the elapsed time of this function. (It can be used many times...)
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------
diary off % Finish the recording of the log file
