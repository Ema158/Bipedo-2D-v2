
function PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters)

% In this code the coefficients of the polynomials that define the trajectory for step of the robot are computed
T = gait_parameters.T;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Q1 CoM Height
a_z = gait_parameters.a_z;  % Maximum amplitude oscillation of the CoM
z_i = gait_parameters.z_i;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Q2 x position free foot
x_ffoot_i = gait_parameters.x_ffoot_i;
x_ffoot_f = gait_parameters.x_ffoot_f;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Q3 z position free foot
H_ffoot = gait_parameters.H_ffoot;
z_ffoot_i = gait_parameters.z_ffoot_i;
z_ffoot_f = gait_parameters.z_ffoot_f;
v_foot_f = gait_parameters.v_foot_f;  % Final landing velocity of the free foot;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Q4 Pitch free foot
Pitch_ffoot_i = gait_parameters.Pitch_ffoot_i;     % Initial rotation in Y of the free foot
Pitch_ffoot_f = gait_parameters.Pitch_ffoot_f;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Q5 Pitch hip
Hip_Pitch_i = gait_parameters.Hip_Pitch_i;
Hip_Pitch_f = gait_parameters.Hip_Pitch_f;
% middle time to maximum high of the free foot
T_midCoMz = gait_parameters.T_midCoMz;
T_midFoot = gait_parameters.T_midFoot;
% INITIAL DESIRED VALUES in position and velocity of the CONTROLLED VARIABLES
% ======================================================================================
% Desired positions at the beginnning of the step (time t=0)
h0_d = zeros(5,1);
h0_d(1) = z_i;
h0_d(2) = x_ffoot_i;
h0_d(3) = z_ffoot_i;
h0_d(4) = Pitch_ffoot_i;
h0_d(5) = Hip_Pitch_i;

% Desired velocities at the beginnning of the step (time t=0)
hp0_d = zeros(5,1);
% CHECK if the transition is taken into account or not to consider the values after impact or the initial desired values
transition = gait_parameters.transition;
if transition
    % The initial positions and velocities are based on the current state of the robot after the transition (impact or not)
    [Qplus, QpPlus] = current_states(robot);
    IniPos = Qplus(1:5); 
    IniVel = QpPlus(1:5);
else % The polynomials are computed for ideal initial conditions 
    % (this is useful for compute the final state of the robot for compute the first step)        
    IniPos = h0_d;
    IniVel = hp0_d;
end

% FINAL DESIRED VALUES in position and velocity of the CONTROLLED VARIABLES
% ======================================================================================
% The Desired values for position of the controlled variables "hd" at the beginnning of the step (time t=T) are the 
% same that the ideal ones at the beginning of the step for almost all the variables, except for:
% -----------------------------------------------------------------------------
hT_d = h0_d;
hT_d(1) = z_i;
hT_d(2) = x_ffoot_f;
hT_d(3) = z_ffoot_f;
hT_d(4) = Pitch_ffoot_f;
hT_d(5) = Hip_Pitch_f;
% Desired values for velocity of the controlled variables "hd" at the end of the step (time t=T)
% -----------------------------------------------------------------------------
hpT_d = zeros(5,1);
hpT_d(4) = v_foot_f;

% The Coefficients of 5th order polynomials for all the controlled variables are computed (except for the first and 
% fourth ones since they are computed after this)
for i=[2,4:5]
    PosD = [0,IniPos(i);
            T, hT_d(i)];
    VelD = [0, IniVel(i);
            T, hpT_d(i)];
   PolyCoeff.(['hd', int2str(i)]) = findPolyCoeff(PosD,VelD,[]); % posd,veld,accd
end
% However since it is desired to create 6 order polinomials for the vertical evolution of the CoM and Free foot,
% (due to an intermediate desired value) the coefficients for these polynomals are computed individually: 
PosD = [0,IniPos(1);
        T_midCoMz, z_i+a_z;      %<-- basically due to this calculation was not included in the loop
        T, hT_d(1)];
VelD = [0, IniVel(1);
        T_midCoMz, 0;
        T, hpT_d(1)];    
PolyCoeff.hd1 = findPolyCoeff(PosD,VelD,[]); % posd,veld,accd
% Desired vertical displacement of the free foot
PosD = [0,IniPos(3); % z_ffoot_i = 0;
        T_midFoot, H_ffoot;      %<-- basically due to this calculation was not included in the loop
        T, hT_d(3)]; % 
VelD = [0, IniVel(3);
        T_midFoot, 0;
        T, hpT_d(3)];    
PolyCoeff.hd3 = findPolyCoeff(PosD,VelD,[]); % posd,veld,accd

