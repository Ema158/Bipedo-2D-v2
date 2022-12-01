function   Biped_param = SS_bipedo_StoppingSS()

% Parameters for a walking gait base on TIME:
%% Gait parameters 
% ============================
T = 0.5;         % Time step
S = 0.05;         % Step length 0.1
a_z = 0;      % Maximum amplitude oscillation of the CoM
H_ffoot = 0.02;  % Foot maximum hight 
% v_foot_f = -0.1; % Final landing velocity of the free foot;
v_foot_f = 0; % Final landing velocity of the free foot;

z0 = 0.10;       % hight of the CoM 0.27
g = 9.81;        % acceleration of gravity
% --------------
% Torso
% Torso_Pitch_i = 0;%q4 Q7
% Torso_Pitch_f = deg2rad(0);%q4 Q7
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%Tiempos
T_midFoot=T/2;
T_midCoMz=T/2;
%% Desired ZMP
% ---------------------------------------------------------------------
% LOCAL Desired EVOLUTION of the ZMP (w.r.t. the Support foot point)
% For making the evolution of the ZMP symmetric
T1 = 0; % This two times are used to build the trajectory of the ZMP and are used in "dynm_HZD.m" 
T2 = T; % So the trayectory will be fixed from 0 to T1, then the motion will be from T1 to T2 and then fixed from T2 to T 
ZMPxIni =  0;      % Local position of the desired ZMP in X for each step
ZMPxMid =  -0.006038523579359;      % Local position of the desired ZMP in X for each step
ZMPxEnd =  0.001769957857284;
Pos = [T1 ZMPxIni; 
       T/2 ZMPxMid;
       T2 ZMPxEnd];
Vel = [];
Acc = [];
ZMPxCoeff = findPolyCoeff(Pos,Vel,Acc);

% Parametres for evolution of the free foot
gait_parameters.T = T;               % Period of the step
gait_parameters.g = g;               % gravity acceleration
% CoM
gait_parameters.z_i = z0;            % Initial hight of the CoM
gait_parameters.a_z = a_z;           % Maximum amplitude oscillation of the CoM
gait_parameters.S = S;               % Half step length
% Free foot desired position and final landing velocity
gait_parameters.x_ffoot_i = -S;      % Initial step position in X
gait_parameters.x_ffoot_f = 0;   % Distance traveled in X by the free foot to reach the final position in X
gait_parameters.z_ffoot_i = 0;       % Initial step position in Z
gait_parameters.z_ffoot_f = 0;       % Final step position in Z
gait_parameters.H_ffoot = H_ffoot;   % Foot maximum hight 
gait_parameters.v_foot_f = v_foot_f; % Final landing velocity of the free foot;
% Free foot desired orientation
gait_parameters.Pitch_ffoot_i = deg2rad(0);   % Initial rotation in Y of the free foot
gait_parameters.Pitch_ffoot_f= deg2rad(0);  % Rotation in Y to be performed by the free foot to reach its final orientation in Y
% ZMP
gait_parameters.Tini = T1;           % Time at which the ZMP will start to move (it should be >= 0)
gait_parameters.Tend = T2;           % Time at which the ZMP will stop its motion (it should be <= T)
gait_parameters.ZMPxIni = ZMPxIni;   % Initial desired position in X
gait_parameters.ZMPxMid = ZMPxMid;   % Mid desired position in X
gait_parameters.ZMPxEnd = ZMPxEnd;   % Final desired position in X
gait_parameters.ZMPxCoeff = ZMPxCoeff; % Coefficients for the polynomial trayectory of the ZMP in X
% Hip pitch
gait_parameters.Hip_Pitch_i = 0;   % Initial orientation of the hip around Y
gait_parameters.Hip_Pitch_f = deg2rad(0);  % Final orientation of the hip around Y 
%%Tiempos
gait_parameters.T_midFoot=T_midFoot;
gait_parameters.T_midCoMz=T_midCoMz;
%% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
% Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT can be considered. Files: "hd_Polyn", "hpd_Polyn_t" and "hppd_Polyn_t". 
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t". 
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT can be considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x". 
OptionContVar = 1;
% -------------------------------------------------------------------------------------------------
%% Cyclic motion (Dx,xpf) and optimized parameters
% Rcyc = [0.004788336495620,-0.006235222545144];
Rcyc = [-0.005935911772509,0.161826846244097];
% Creating a structure for the parameters
Biped_param.gait_parameters = gait_parameters;
Biped_param.ControlledVariableOption = OptionContVar;
Biped_param.Rcyc = Rcyc;
