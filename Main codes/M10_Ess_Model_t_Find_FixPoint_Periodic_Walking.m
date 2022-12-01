close all;
clear all;
clc
% ---------------------------------------------------------------------------------------------
currentfolder = pwd; % save current path
cd ..                % go one folder down (go out of the current folder and stay in the previous one)
add_paths;           % add all folders where all files are founded in order to acces to them
cd(currentfolder);   % return to the original path
% ---------------------------------------------------------------------------------------------

% GENERAL OPTIONS
% ----------------------------------------------------
DataName = 'FixPoint_Param_Bipedo_only_SS';
% ----------------------------------------------------

diary(['- NAOlog-',DataName,'.txt']) % Create a .txt file to store all display information
diary on % Start the recording of the log file
echo on
% Fixed Point for the Essential model
% ==============================
% Creation: 18/ago/2018
% Last modification: --/--/----
% -----------------------------------------------------------------------------------------------------------
% Finding a Fix point (which means a Cyclic motion for the Essential model) by considering SS and DS phases
% -----------------------------------------------------------------------------------------------------------
% This code is based on "M13b_EssentialModelfindingFixedPointZMPvar_t"
%  *     *     *     *     *     *     *     *     *     *     *     *     *     *     *     * 
% IMPORTANT NOTE: % CHECK which controlled variables are you using "h_d". Files to check:
%               "JointsPosVel_from_CoMPosVel_HZDtime",
%               "joints_fromCoMpos_Vel_HZDtime" 
%               "Desired_qfpp_HZDtime"
%               "InvGeometricHZDtime"
%  *     *     *     *     *     *     *     *     *     *     *     *     *     *     *     * 
% ------------------------------------------------------------------------------------------------------------
echo off;


% global gait_parameters
global robot 
global gait_parameters
% coms=1;
% Parameters
% ---------------------------------------------------------
% Nao_param = Param00_ZMPx_var();
% Nao_param = SSParam09d_ZMPx_var_v2();
Bipedo_param = Param01_2DBiped_PeriodicGait_ZMP0();
% Nao_param = SSParam12_ZMPx_var();
% Nao_param = SSParam11_ZMPx_var_salto_b();
% Nao_param = SSParam13_ZMPx_var_Internal_State_x_prueba_t();
% Nao_param = SSParam13_ZMPx_var_();
% ---------------------------------------------------------
gait_parameters = Bipedo_param.gait_parameters;
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
OptionContVar = Bipedo_param.ControlledVariableOption;
% -------------------------------------------------------------------------------------------------

% FINAL PROPOSED values for the velocity of the CoM
% Cyclic motions:
% -------------------------------------------------------------------------------------------------------------------
Rcyc = Bipedo_param.Rcyc;

% If NO error in the velocity is desired, chose "Error = 0"
Error = 0; % Percentage of error   <-------
% ----------------------------------
Dx = Rcyc(1);
xpf = Rcyc(2);

% Initial and final position is computed as (this is not used in here)
% x0 = -S/2 + Dx;
% y0 = D/2 - Dy;
% xf = S/2 + Dx;
% yf = D/2 + Dy;

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

% OPTION - For plotting of the evolution of the CoM in "PEvents.m" file (while the solver is working on the dynamics)
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
% Parameters for visualization (in the command display)
%----------------------------------------------------------
global contA   % Variable to count the number of cycles performed to find the fixed point
contA = 1;     % it is also used to create new figures to see the evolution of the CoM when contC is defined 
% -----------------------------------------------------

% ====================================================================


%% Continue
% Variable that is going to be founded 
global scale
scale = 1; %1000 (2-2)
R0 = [scale*Dx xpf];

optfmin = optimset('MaxFunEvals',1200,'TolFun',1e-11,'TolX',1e-11,'Algorithm','levenberg-marquardt'); % 'trust-region-dogleg', 'levenberg-marquardt' , 'trust-region'
[fix,Func] = fsolve('CycleEssentialModelZMPvar_SSphase_t',R0,optfmin);

fix(1) = fix(1)/scale;
fix(2) = fix(2)/scale;
% Results
% ============
disp('Final results')
disp('=========================================')
fprintf('The value for R* = [Dx, Dy, xpf*, ypf*] is [%e,%e]\n',fix)
fprintf('The value of the Vectorial function in R* is [%e,%e]\n',Func)


%% Plotting results
% =====================================================================
global Rtotal FixPoint errorLanding
numCyc = 1:length(FixPoint);
% ----------------
figure
plot(numCyc,errorLanding)
ylabel('error landing foot')
xlabel('Cycles (0-> Landing OK. 1-> No landing)')
figure
plot(numCyc,FixPoint)
hold on
plot(numCyc,0*numCyc,'--r');
ylabel('Fix point function')
xlabel('cycles')
figure
plot(numCyc,Rtotal)
hold on
plot(numCyc,0*numCyc,'--r');
ylabel('Parameters')
xlabel('cycles')

%% Storing all the parameters and information of this simulation
% =====================================================================
disp(['Saving data as: Data_' DataName]);
disp('------------------------------');
save(['Data_',DataName])


%% End of the code
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------
diary off % Finish the recording of the log file