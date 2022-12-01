function [ Im ] = actuator_inertia
%Motor inertias generated from NAO
%http://doc.aldebaran.com/2-1/family/nao_h25/motors_h25_v4.html
%https://www.portescap.com/sites/default/files/ed_about_brush_dc_motors.pdf
%Taking into account the rotor inertia and the reudction ratio

%Rotor inertia
Ir = [3;...q1
    3;...q2
    3;...q3
    3;...q4
    3;...q5
    3;...q6
    3];...q7
 
Ir = Ir*1e-7;

%Reduction ratios
N = [201.3;...q1
    130.85;...q2
    130.85;...q3
    130.85;...q4
    201.3;...q5
    201.3;...q6
    201.3];...q7
    
Im = Ir.*N.^2;

% % ==============================================================================================
% % The NEXT part of the code is added in order to concentrate the mass of the robot in one point
% % ==============================================================================================
global alphaM % alphaM = [0-1]. Allow to remove gradually the value of all masses and inertias of the robot
% alphaM = 0 -> all masses are 0 -> all mass is concentrated in one point
% alphaM = 1 -> all masses are normal -> all masses are distributed
% If alphaM is empty it means it was not initialized by other codes, therefore it is not required to do this 
if ~isempty(alphaM) 
    if alphaM > 1
        alphaM = 1;
    elseif alphaM < 0
        alphaM = 0;
    end           
    Im=alphaM*Im;
end

