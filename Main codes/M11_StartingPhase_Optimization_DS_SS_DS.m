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
DataName = 'Optimization_StartingPhase_Parame13b_reduced';
% ----------------------------------------------------
diary(['- NAOlog-',DataName,'.txt']) % Create a .txt file to store all display information
diary on % Start the recording of the log file
echo on

% Finding the ZMP points to achieve a desired fixed point by using otimization with the Essential model
% ==============================
% Creation: 11/sep/2018
% Last modification: --/--/----
% -----------------------------------------------------------------------
% Finding the ZMP points to achieve a desired fixed point.
% -----------------------------------------------------------------------
% The walking gait start in rest with double support, then single support and finishes in DS again.
% This code is based on "M71_StartingPhase_Optimization_DS_SS_DS" but instead of 9 parameters to optimize, there are only 4
% 1-> ZMP in X  for initial, intermediate and final in DS AND initial of SS
% 2-> ZMP in Y  for final in DS initial of SS
% 3-> ZMP in X  for final of SS and initial of DS_final
% 4-> ZMP in Y  for final of SS and initial of DS_final
% Note: The evolution of the ZMP for the 3 phases are straight lines. Moreover for DS is a vertical line. In fact:
%       Initial ZMP in Y for DS is fixed
%       The middle point for DS is just considered in Y and is fixed
%       Final ZMP in X for DS_final is fixed   
%       No middle points are taken into account for SS and DS_final
%
% ------------------------------------------------------------------------------------------------------------
echo off
global robot

% Parameters
% ---------------------------------------------------------
%
Biped_param_DS = SS_bipedo_StartingDS();
Biped_param_SS = SS_bipedo_StartingSS();
Biped_param_DS_final = SS_bipedo_StartingDS_final();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global gait_parameters_SS gait_parameters_DS gait_parameters_DS_final
gait_parameters_SS = Biped_param_SS.gait_parameters;
gait_parameters_DS = Biped_param_DS.gait_parameters;
gait_parameters_DS_final = Biped_param_DS_final.gait_parameters;

% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
global OptionContVar  % Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_t" and "hppd_Polyn_t". 
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t". 
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x". 
OptionContVar = 1;
% -------------------------------------------------------------------------------------------------

% Desired Cyclic motion
% -------------------------------------------------------------------------------------------------------------------
Rcyc = Biped_param_DS_final.Rcyc;
Dx = Rcyc(1);
xpf = Rcyc(2);

S = gait_parameters_DS_final.S;
% Initial and final position is computed as 
x0 = -S/2 + Dx;
xf = S/2 + Dx;

global DesFixedPoint
DesFixedPoint = [xf,xpf];

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
ZMPx_1 = gait_parameters_SS.ZMPxIni; % or gait_parameters_DS.ZMPxEnd;
ZMPx_2 = gait_parameters_SS.ZMPxEnd; % or gait_parameters_DS.ZMPyend;

R0(1) = ZMPx_1; % Proposed ZMP in X for initial, intermediate and final in DS AND initial of SS
R0(2) = ZMPx_2; % Proposed ZMP in X for final of SS and initial of DS_final

% The initial and final points of the ZMP in X and Y are all restricted to the size of the support foot
% lim_ZMPx = [-0.02; 0.08]; 
% Limits for the optimized parameters
% ---------------------------------------          
% LowerBounds = [lim_ZMPx(1), lim_ZMPy(1),  lim_ZMPx(1), lim_ZMPy(1)];
% LowerBounds = [lim_ZMPx(1), lim_ZMPy(1),  lim_ZMPx(1), lim_ZMPy(1)];
% UpperBounds = [lim_ZMPx(2), lim_ZMPy(2),  lim_ZMPx(2), lim_ZMPy(2)];

% for "fmincon"
% Algorithm: 'active-set', 'interior-point', 'sqp', 'trust-region-reflective', or 'sqp-legacy'.
% options = optimoptions(@fmincon,'Display','iter','TolFun',1e-11,'TolX',1e-11,'MaxFunEvals',60000,'MaxIter',700,'Algorithm','active-set'); % GOOD
% options = optimoptions(@fmincon,'Display','iter','TolFun',1e-11,'TolX',1e-11,'MaxFunEvals',60000,'MaxIter',700,'Algorithm','sqp'); % GOOD
% % [R,Func,exitflag,output,lambda,grad,hessian] = fmincon('Optim_ObjectiveFunc_Starting_DS_SS_DS',R0,[],[],[],[],LowerBounds,UpperBounds,'Constraints',options);
% [R,Func,exitflag,output,lambda,grad,hessian] = fmincon('Optim_ObjectiveFunc_Starting_DS_SS_DS_reduced',R0,[],[],[],[],LowerBounds,UpperBounds,[],options);

optfmin = optimset('MaxFunEvals',1200,'TolFun',1e-11,'TolX',1e-11,'Algorithm','levenberg-marquardt'); % 'trust-region-dogleg', 'levenberg-marquardt' , 'trust-region'
[fix,Func] = fsolve('Optim_ObjectiveFunc_Starting_DS_SS_DS_fsolve',R0,optfmin);


% Results
% ============
disp('Final results')
disp('=========================================')
fprintf('The value for R* = [ZMpx_i, ZMPx_f] is [%e,%e]\n',fix)
fprintf('The value of the Vectorial function in R* is [%e,%e]\n',Func)

%% Plotting results
% =====================================================================
% global ObjecFunc Rtotal
% global errorLanding errorWorkspace
% numCyc = 1:length(ObjecFunc);
% % ----------------
% figure
% subplot(2,1,1)
% plot(numCyc,ObjecFunc)
% ylabel('J')
% xlabel('cycles')
% subplot(2,1,2)
% plot(numCyc,errorLanding)
% hold on
% plot(numCyc,errorWorkspace,'k')
% ylabel('0-> OK. 1-> Error')
% xlabel('Cycles. Error landing foot (blue) or OutWorkspace (black)')
% 
% figure
% plot(numCyc,Rtotal)
% ylabel('Optimized parameters')
% xlabel('cycles')

%% Storing all the parameters and information of this simulation
% =====================================================================
% Optimization.ParamVectR = R;
% Optimization.FuncValue = Func;
% fminconData.exitflag = exitflag;
% fminconData.output = output;
% fminconData.lambda = lambda;
% fminconData.grad = grad;
% fminconData.hessian = hessian;
% Optimization.fminconData = fminconData;
% Build just ONE structure with all the information
InfBiped.robot = robot;
global gait_parameters 
InfBiped.gait_parameters = gait_parameters;
InfBiped.gait_parameters_SS = gait_parameters_SS;
InfBiped.gait_parameters_DS = gait_parameters_DS;
InfBiped.gait_parameters_DS_final = gait_parameters_DS_final;
% InfNAO.Optimization = Optimization;
disp(['Saving all workspace data as: Data_' DataName]);
disp('------------------------------');
save(['Data_',DataName])

%% End of the code
toc; % Finishing of counting the elapsed time of this function. (It can be used many times...)
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------
diary off % Finish the recording of the log file