function dhd_Phi = dhd_Phi_Polyn(gait_parameters,Phi)
% % This is the partial dereivative of "hd" with respect to vector "Phi = [qf^T phi^T]".
% Remember that..

PolyCoeff = gait_parameters.PolyCoeff;
x = Phi(1);
% Al this vectors are used later (in other files) to compute the desired velocity
% ---------------------------------------------------------------
% dhd_x - > is the partial derivative of vector "h_d" w.r.t. "x"
% ------------------------
dhd_x = zeros(5,1);
for i=1:5
    dhd_x(i) =  polyval(polyder(PolyCoeff.(['hd', int2str(i)]) ),x);
end
% dhd_y - > is the partial derivative of vector "h_d" w.r.t. "y"

dhd_Phi = dhd_x;