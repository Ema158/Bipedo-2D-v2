function dPhi_dhd_x = dPhi_dhd_x_Polyn(gait_parameters,Phi)

x = Phi(1);
PolyCoeff = gait_parameters.PolyCoeff;

% Al this vectors are used later (in other files) and along with other files if necesary (dPhi_dhd_y, etc)
% to compute the desired acceleration
% ---------------------------------------------------------------
dx_dhd_x = zeros(5,1);
for i=1:5
    dx_dhd_x(i) =   polyval(polyder(polyder(PolyCoeff.(['hd', int2str(i)]) )),x);
end
% Since all trajectories depends only on "x", the derivative w.r.t. "y" is zero
dPhi_dhd_x = dx_dhd_x;