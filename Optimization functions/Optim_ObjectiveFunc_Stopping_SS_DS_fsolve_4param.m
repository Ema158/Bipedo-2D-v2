function Func = Optim_ObjectiveFunc_Stopping_SS_DS_fsolve_4param(R)

global gait_parameters_SS_Stopping gait_parameters_DS_Stopping gait_parameters_previous_DS Rcyc
global DesFixedPoint 
% ==============================================================================================
global contA % To count the number of cycles performed
Hour = datestr(now,13); % Now ->Read the current hour and date, 13 -> store just the hour, -1 -> default...
fprintf('\n-------------- OBJECTIVE FUNCTION FILE -------------------\n')
fprintf([Hour ' -> Cycle k = %d (of 700 maximum) for the EssentialModel \n'],contA);
% ==============================================================================================
disp('Performing one step of the robot');
% -----------------------------------------------   

ZMPx_SS_End = R(1); % 
ZMPx_SS_Mid = R(2);

fprintf('Proposed ZMP for final SS:  \nZMPx = %f.\n',ZMPx_SS_End)
fprintf('Proposed ZMP for Mid DS: \nZMPxMid_DS = %f.\n.',ZMPx_SS_Mid)
fprintf('-----------------------------------------------------\n')

%% Desired ZMP
% ---------------------------------------------------------------------
% Updating parameters for ZMP in the initial SS phase
T_SS = gait_parameters_SS_Stopping.T;
T1 = gait_parameters_SS_Stopping.Tini;
T2 = gait_parameters_SS_Stopping.Tend;
ZMPx_Ini = gait_parameters_SS_Stopping.ZMPxIni;  % Fixed value
ZMPx_Fin = ZMPx_SS_End;
% Time at which the ZMP will stop its motion (it should be <= T)
Pos = [T1 ZMPx_Ini;
       T_SS/2 ZMPx_SS_Mid;
       T2 ZMPx_Fin];
Vel = [];
Acc = [];
ZMPxCoeff = findPolyCoeff(Pos,Vel,Acc);
gait_parameters_SS_Stopping.ZMPxCoeff = ZMPxCoeff; % Coefficients for the polynomial trayectory of the ZMP in X
gait_parameters_SS_Stopping.ZMPxIni = ZMPx_Ini;   % Initial desired position in X
gait_parameters_SS_Stopping.ZMPxEnd = ZMPx_Fin;   % Final desired position in X

% ---------------------------------------------------------------------
% Updating parameters for ZMP in the initial DS phase
T_DS = gait_parameters_DS_Stopping.T;
T1 = gait_parameters_DS_Stopping.Tini;
T2 = gait_parameters_DS_Stopping.Tend;
ZMPx_DS_End = gait_parameters_DS_Stopping.ZMPxEnd;
ZMPx_DS_Mid = gait_parameters_DS_Stopping.ZMPxMid;
% T_SS = gait_parameters_SS_Stopping.T;
% T_DS = gait_parameters_DS.T;
% Time at which the ZMP will stop its motion (it should be <= T)
Pos = [T1 ZMPx_SS_End;
%        T_DS/2 ZMPx_DS_Mid;
       T2 ZMPx_DS_End];
Vel = [];
Acc = [];
ZMPxCoeff = findPolyCoeff(Pos,Vel,Acc);
gait_parameters_DS_Stopping.ZMPxCoeff = ZMPxCoeff; % Coefficients for the polynomial trayectory of the ZMP in X
gait_parameters_DS_Stopping.ZMPxIni = ZMPx_SS_End;   % Initial desired position in X
gait_parameters_DS_Stopping.ZMPxMid = ZMPx_DS_Mid;   % Final desired position in X
gait_parameters_DS_Stopping.ZMPxEnd = ZMPx_DS_End;   % Final desired position in X
% ---------------------------------------------------------------------
Dx = Rcyc(1);
xpf = Rcyc(2);
xf = (gait_parameters_previous_DS.S)/2 + Dx;
x0 = -(gait_parameters_previous_DS.S)/2 + Dx;
xp0 = Rcyc(2);
% X_final = [xf;yf;xpf;ypf];
X_final = [x0;xp0];
%% Robot initial step DS and SS (Starting from REST)
[t,Xt,t_SS,Xt_SS,t_DS,Xt_DS,gait_parameters_SS_Stopping,gait_parameters_DS_Stopping,] = ...
    robot_step_EssModel_Stopping_SS_DS_t(X_final,gait_parameters_SS_Stopping,gait_parameters_DS_Stopping);
% Initial states (current step) after impact
x0 = Xt(1,1);
xp0 = Xt(1,2);
% Final states (current step) after impact
xf = Xt(end,1);
xpf = Xt(end,2);
xf_fix = DesFixedPoint(1);
xpf_fix = DesFixedPoint(2);
fprintf('Desired cyclic motion at the end DS phase = [%f,%f]\n',xf_fix,xpf_fix)
fprintf('Initial states Step {k} = [%f,%f]\n',x0,xp0)
fprintf('Final states Step {k} = [%f,%f]\n',xf,xpf)

%% ERRORS (for skip the rest of the function)
% -------------------------------------
global contD OutOfWorkSpace errorWorkspace
if OutOfWorkSpace
    contD = []; % Every time the robot is out of the workspace and "PEvents" is called, 'contD' is used to stop de integration
    OutOfWorkSpace = []; % Reset workspace flag
    errorMessage = 'OUT OF WORKSPACE -> ';
    errorWorkspace(contA) = 1;
else
    errorWorkspace(contA) = 0;
end
% -------------------------------------
global noLanding errorLanding % is defined in "PEvents_HDZtime."
if (isempty(noLanding) || noLanding==0)    % if there was an impact...
    errorLanding(contA) = 0;
else   % if there was NO impact
    errorLanding(contA) = 1;        
    noLanding = 0; % Restart flag
    errorMessage = 'NO LANDING -> ';     
end
ERROR = errorLanding(contA) || errorWorkspace(contA);
% ================================================================================

% Some of these global functions are used inside the IF-ELSE sentence
if ~ERROR    % IF THERE IS NO ERROR... the code continues....
    %% Sampling
    errorMessage = '';    
    
    %% Objective function    
    Func = [xf_fix - xf, xpf_fix - xpf]; 
    Tk = t(end); % Elapsed time to produce the impact
% FUNCTION TO MINIMIZE:
% ----------------------------------------------
 fprintf('Desired time step T = %f. Real time Step at cycle %d, Tk = %f\n',T_SS + T_DS,contA,Tk)
    fprintf('Proposed final values for the CoM states are: \n[xf(k), xpf(k)] = [%f,%f]\n',xf_fix,xpf_fix)
%     fprintf('After impact the values for the CoM states are: \n[x0(k+1), y0(k+1), xp0(k+1), yp0(k+1)] = [%f,%f,%f,%f]\n',x0, y0, xp0, yp0)
    fprintf('The final values for the CoM states are: \n[x(T)(k+1), xp(T)(k+1)] = [%f,%f]\n',xf,xpf)
%     fprintf('Function "[xf(k) - xf(k+1),yf(k) - yf(k+1), xpf(k) - xpf(k+1),ypf(k) - ypf(k+1), fac*(T - T(k))]"\n         = [%e,%e,%e,%e,%e]\n',Func)
%     fprintf('with fac = %d\n',fac)    
    fprintf('Function "[xf(k) - xf(k+1), xpf(k) - xpf(k+1)]"\n         = [%e,%e]\n',Func)

else
    disp([errorMessage,' Integration stopped because the robot configuration is unaccesible (Check PEvents_HDZtime.m)']);
    disp('--------------------------------------------------------------------------------------------');        
    Func = [1000,1000]; 
    disp('NO COMPUTATION OF TORQUES was performed. A high value for cost function was defined....');    
    disp('--------------------------------------------------------------------------------------------');    
end

%% Storing variables
% ----------------------------------
% global ObjecFunc Rtotal
% ObjecFunc(contA) = J;
% Rtotal(:,contA)  = R;


%% To display information in the command display
% ----------------------------------------------------------------
contA = contA + 1;
global contB % number of iteration inside the solver
if ~isempty(contB)
    contB = 1;
end
    
global contC 
if ~isempty(contC)  % If contC is empty means it wasn't initialized (in other codes) thus it is not required
                    % to plot the evolution of the CoM in PEvents
    contC = 1;  % If is not empty, then each cycle is re-initialized and the variables must be cleaned
    global CoMx CoMy
    CoMx = [];
    CoMy = [];        
end
% -----------------------------------------------------------------

