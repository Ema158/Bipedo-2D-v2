function q = InvGeometricHZDtime(x,robot,gait_parameters,t)
%Numerical solver of the inverse geometric model assuming the virtual
%constraint are respected.
q_init=robot.q;
Case = 'position';
% "Case", "gait_parameters" and "t" are used inside "OptionDesiredTrajectory"
OptionDesiredTrajectory;

G = robot.CoM;  % Posiciï¿½n X,Y,Z del COM del robot
Q = [state_v(robot);G(1)];  % Conjunto de posiciones OPERACIONALES reales Q = [h(q)^T, q_f^T]^T
Qdes = [hd; x];  % Qd - > Desired controlled variables set and the CoM

%Numerical threshold
Thre = 0.2;
global nQ
% ---------------------------
dQ = Qdes-Q;
nQ = max(abs(dQ));
i=0;
Analyze = false;  % 1-> Plot the robot each iteration, so we can analize who the algorithm is converging 0-> We don't analyze anything
if Analyze
    % Just to check how the algorithm adjust the position step by step until reach the desired position
    % -----------------------------------------------------
    % Posición incial
    robot = robot_move(robot,q_init);
    figure
    robot_draw(robot,0,0);
    axis equal
    % pause;
    figure
    % ------------------------
end

while nQ>1e-10
    if nQ>Thre
        dQ = dQ./norm(Q).*Thre;
    end
    dq = InvKinematic(dQ,robot);
    
    %Next iteration
    q=robot.q+dq;

    robot = robot_move(robot,q);
    G = robot.CoM;
    Q = [state_v(robot);G(1)]; % Posiciones operacionales reales
    dQ = Qdes-Q;
    nQ = max(abs(dQ));
    i=i+1;
    if Analyze
        % Just to check how the algorithm adjust the position step by step until reach the desired position 
        % -----------------------------------------------------
        i
        hold off
        plot(0,0)
        hold off
        robot_draw(robot,0,0);
        axis equal
        view(3)
        fprintf('Ankle Pitch ->  q1 = %f, q7 = %f \n',rad2deg(q(1)),rad2deg(q(6)))
        fprintf('knee Pitch  ->  q2 = %f, q6 = %f \n',rad2deg(q(2)),rad2deg(q(5)))
        fprintf('Hip Pitch   ->  q3 = %f,  q4 = %f \n',rad2deg(q(3)),rad2deg(q(4)))
        [[1:6]' Qdes Q]
        pause;
        % ------------------------
    end
    if i == 200
        warning('Out of the workspace!');
        global OutOfWorkSpace
        if isempty(OutOfWorkSpace)
            OutOfWorkSpace = 1;
        end
        figure(99)
        title('Out of workspace');
        [[1:6]' Qdes Q];
        nQ; 
        robot_draw(robot,0,0);
        robot = robot_move(robot,q_init);
        disp('Robot configuration re-initialized')
%         figure(100)
%          robot_draw(robot,0,0);
% %         pause;
        break
    end
end
q=robot.q;
%Test
%  figure;
%  cla;
%  robot_draw(robot,0,0);
%  q
%  pause

end

