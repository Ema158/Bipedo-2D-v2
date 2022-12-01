function [CoM,J_CoM,J_Ankle,crossM,J_CoMs] = compute_com(robot,PI)

%% CoM position/velocity

%Mass informations
M = PI.masse;
CoM_j = PI.CoM;
%///////////////////////////////////////////////////////////////////////////
%Initialisations
CoM = [0;0;0];
J_CoMs = zeros(3,robot.joints,8);
J_CoM = zeros(3,robot.joints);
T = robot.T;
ant =       [0,... 1
             1,... 2
             2,... 3
             3,... 4
             4,... 5
             5,... 6
             6,... 7
             7]; %8  
act= [0,1,2,3,4,5,6,0];
crossM = zeros(3,3,8);

%Computation of the cross matrices
for i = 1:8
crossM(:,:,i) = cross_matrix(T(1:3,3,i)); % Transforma el vector "a" de la matriz T = [s n a p; 0 0 0 1]; en una matriz antisimétrica
end
for j = 1 : 8
    if M(j) ~=0
        %Center of mass position (X,Y)
        MS_j = T(1:3,:,j) * [CoM_j{j};1]; % Este es el vector de posicion del CoM_j respecto al marco 0
        CoM = CoM + M(j) * MS_j;         % De igual forma, aquí se va calculando el CoM total del robot "Sum(j=1:36) m* ^0Com_j"
        %Center of mass velocity (XD,YD)
        J_X = zeros(3,robot.joints);      % Se crea una matriz de 3x27 para calcular J_CoM_j
        F = j;% Se empiezan a crear los jacobianos para cada marco( los que no tienen masa asignada seran una matriz de ceros)
        while F~=1    % ¿Llegamos al marco 1? Si si, sale del "while", si no, continúa... (F nunca es menor que 1)
            % Aqui se va calculando cada columna de la matriz J_CoM_j. Nótese que se "brinca" las columnas que
            % corresponden a los marcos que NO tienen asignada una masa.. Nótese que los jacobianos empiezan de ADELANTE pa'trás
            if act(F)~=0 % ¿El marco F tiene una articulacion (q)? Si si, entra al "if", si no no =P
               J_X(:,act(F)) =crossM(:,:,F)*(MS_j-T(1:3,4,F)); %col  -> 0a_j X (0Pcom_j - 0P_j)
            end
            F = ant(F);
        end
        J_CoMs(:,:,j)=J_X;               % Se asigna 
        J_CoM = J_CoM+M(j)*J_X;
     end
end

CoM = CoM / robot.mass;   % = 1/mT Sum_{i=1}^n a_j X ^jCom_j   donde  jCom_j es el vector constante de CoM del del eslabon j 
                          % respecto al marco j, y a_j es el vector de la matriz "snap" del marco "j"
J_CoM = J_CoM/robot.mass;

%% J_Ankle is the jacobian of the ankle for x,y position and leg tip for z
% position
F = 7;
J_Ankle = zeros(3,robot.joints);
while F~=1
        if act(F)~=0 
            J = crossM(:,:,F)*(T(1:3,4,7)-T(1:3,4,F));
            J_Ankle(1:2,act(F)) =J(1:2);
        end
        F = ant(F);
end
F = 8;
while F~=1
        if act(F)~=0 
            J = crossM(:,:,F)*(T(1:3,4,8)-T(1:3,4,F));
            J_Ankle(3,act(F)) =J(3);
        end
        F = ant(F);
end








