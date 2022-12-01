function qD = InvKinematic(Qp,robot)
%Inverse Kinematic from Q parameter variables 

J_G=robot.J_CoM; % Jacobiana del CoM del robot
J = [J_state_v(robot);J_G(1,:)]; 
qD = J\Qp;

end

