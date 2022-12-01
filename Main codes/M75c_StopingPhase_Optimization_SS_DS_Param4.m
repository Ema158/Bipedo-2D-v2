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
DataName = 'Optimization_StoppingPhase_Param05_2Param';
% ----------------------------------------------------
diary(['- Bipedlog-',DataName,'.txt']) % Create a .txt file to store all display information
diary on % Start the recording of the log file
echo on

% Finding the ZMP points to achieve a desired fixed point by using optimization with the Essential model
% ==============================
% Creation: 25/may/2021
% Last modification: --/--/----
% -----------------------------------------------------------------------
% Finding the ZMP points to take the robot from the dynamic walking on the fixed point to a rest position (supported on its both feet).
% -----------------------------------------------------------------------
% The walking gait start at the fixed point in single support, then a small step is performed to let both feet aligned and finish in DS.
% This code is based on "M71a_StartingPhase_Optimization_DS_SS_DS_reduced" but in here the optimized parameters are:
% 1-> ZMP in X  for final SS and initial DS
% 2-> ZMP in Y  for final SS and initial DS
% 3-> ZMP in X  for intermediate and final DS
% 4-> ZMP in Y  for intermediate DS 
%
% Note: The evolution of the ZMP for the 2 phases are straight lines. In fact:
%       Initial ZMP in X and Y for SS is fixed
%       Final ZMP in Y for DS is fixed   
%
% ------------------------------------------------------------------------------------------------------------

echo off
global robot

% Parameters
% ---------------------------------------------------------
Biped_param_previous_DS_Phase = SSParamComRob_ZMPx_var();
Biped_param_SS_StoppingPhase = SS_bipedo_StoppingSS();
Biped_param_DS_StoppingPhase = SS_bipedo_StoppingDS();
%   
global gait_parameters_previous_DS gait_parameters_SS_Stopping gait_parameters_DS_Stopping
gait_parameters_previous_DS = Biped_param_previous_DS_Phase.gait_parameters;
gait_parameters_SS_Stopping = Biped_param_SS_StoppingPhase.gait_parameters;
gait_parameters_DS_Stopping = Biped_param_DS_StoppingPhase.gait_parameters;

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
global Rcyc
Rcyc = Biped_param_previous_DS_Phase.Rcyc;
Dx = Rcyc(1);
xpf = Rcyc(2);

S = gait_parameters_previous_DS.S;
% Initial and final position is computed as 
xf = S/2 + Dx;
x0 = -S/2 + Dx;
global DesFixedPoint
DesFixedPoint = [0,0];

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
gait_parameters_SS_Stopping.transition = false; 
% gait_parameters_SS_Stopping.transition = true; %M72c
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
ZMPx_SS_End = gait_parameters_SS_Stopping.ZMPxEnd; % 
ZMPx_SS_Mid = gait_parameters_SS_Stopping.ZMPxMid;

R0(1) = ZMPx_SS_End; % Proposed ZMP in X for initial, intermediate and final in DS AND initial of SS
R0(2) = ZMPx_SS_Mid;

% The initial and final points of the ZMP in X and Y are all restricted to the size of the support foot
% lim_ZMPx = [-0.02; 0.08]; 
% lim_ZMPy = [-0.015; 0.015];
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
[fix,Func] = fsolve('Optim_ObjectiveFunc_Stopping_SS_DS_fsolve_4param',R0,optfmin);


% Results
% ============
disp('Final results')
disp('=========================================')
fprintf('The value for R* = [ZMPx_SS_End, ZMPx_SS_Mid] is [%e,%e]\n',fix)
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
InfBiped.gait_parameters_SS = gait_parameters_SS_Stopping;
InfBiped.gait_parameters_DS = gait_parameters_DS_Stopping;
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