function hpp_d = hppd_Polyn_t(gait_parameters,t)

PolyCoeff = gait_parameters.PolyCoeff;

% Acceleration
% ------------------------
hpp_d = zeros(5,1);
for i=1:5
    hpp_d(i) =   polyval(polyder(polyder(PolyCoeff.(['hd', int2str(i)]) )),t);
end
