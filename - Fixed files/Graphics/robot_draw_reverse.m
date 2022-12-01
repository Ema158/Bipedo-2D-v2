function robot_draw_reverse(robot,x0,y0)
% Modified: 25-jul-2018 -> Victor 
% x0 -> Coordinate of the support foot in X direction
% y0 -> Coordinate of the support foot in Y direction
% alpha -> Orientation of the robot in the plane X-Y
global coms

%Generates a 3D-plot of the robot on second leg
T=robot.T;
ant =       [0,... 1
             1,... 2
             2,... 3
             3,... 4
             4,... 5
             5,... 6
             6,... 7
             7]; %8  
[x,y,z]=sphere;
% % ---------------------------------------------------------------------------  
% supportFrame_Rotated = RotZ_01'*[x0;y0];
% distCoM = robot.CoM(1); 
% suppX = supportFrame_Rotated(1);
% axis([cos(alpha)*(distCoM+suppX)-0.7, cos(alpha)*(distCoM+suppX)+0.7,...
%       sin(alpha)*(distCoM+suppX)-0.7, sin(alpha)*(distCoM+suppX)+0.7, 0 1.4]);
% % ---------------------------------------------------------------------------  
global GlobalCoMX GlobalCoMY
if ~isempty(GlobalCoMX) && ~isempty(GlobalCoMX)
    axis([GlobalCoMX-0.3, GlobalCoMX+0.3,GlobalCoMY-0.3, GlobalCoMY+0.3, 0 0.3]);
else
    axis([-0.4 0.4 -0.4 0.4 -0.1 0.4]);  % For a fixed plot %*
end
% ---------------------------------------------------------------------------  
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;

CoM_Rotated = robot.CoM;
COM = surf(x/150+CoM_Rotated(1)+x0, y/150-CoM_Rotated(2)+y0, z/150+CoM_Rotated(3));
set(COM,'FaceColor',[0 0 0]);
set(COM,'EdgeColor',[0 0 0]);

% % Support foot
% ---------------------------
% 1(xif,y1l) -------------- 3(x1f,y1l) ----------------------->x
%           |              |
%           | 5(x0f,y0f)   |
%           |              |
% 2(xif,y1r) -------------- 4(x1f,y1r)
x0f = x0+T(1,4,2); %*
y0f = y0+T(2,4,2); %*
x1f = T(1,4,1);
xif = T(1,4,2)-0.015; %-0.055
y1r = T(2,4,1)-0.025;
y1l = T(2,4,1)+0.025;
z0f = T(3,4,2);
z1f = T(3,4,1); %*
ver1 = [x0;y0] +[xif;y1l];
ver2 = [x0;y0] + [xif;y1r];
ver3 = [x0;y0] + [x1f;y1l];
ver4 = [x0;y0] + [x1f;y1r];

% Each column (xf,yf and zf) defines 4 vertices to build one polygon
% the first 3 columns define triangles, i.e. the sides of the foot (notice that one vertice is repeated). 
% The last column defines a squared (the foot sole)
xf=[x0f     x0f     x0f     x0f     ver1(1);
    ver1(1) ver1(1) ver3(1) ver2(1) ver2(1);
    ver2(1) ver3(1) ver4(1) ver4(1) ver3(1);
    x0f     x0f     x0f     x0f     ver4(1)];

yf=[y0f     y0f     y0f     y0f     ver1(2);
    ver1(2) ver1(2) ver3(2) ver2(2) ver2(2);
    ver2(2) ver3(2) ver4(2) ver4(2) ver3(2);
    y0f     y0f     y0f     y0f     ver4(2)];

zf=[z0f     z0f     z0f     z0f     z1f;
    z1f     z1f     z1f     z1f     z1f;
    z1f     z1f     z1f     z1f     z1f;
    z0f     z0f     z0f     z0f     z1f];
h=patch(xf,yf,zf,'r');  % Poligons to build the support foot
set(h,'edgecolor','k');

%Support leg
for i = 3:4
    ext1 = T(1:3,4,i)+[x0;-y0;0];
    ext1(2)=-ext1(2);
    surf(x/200+ext1(1),y/200+ext1(2),z/200+ext1(3));

    ext2 = T(1:3,4,ant(i))+[x0;-y0;0];
    ext2(2)=-ext2(2);
    plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'r','LineWidth',2);
end

%Swing leg
for i = 6:7
    ext1 = T(1:3,4,i)+[x0;-y0;0];
    ext1(2)=-ext1(2);
    surf(x/200+ext1(1),y/200+ext1(2),z/200+ext1(3));

    ext2 = T(1:3,4,ant(i))+[x0;-y0;0];
    ext2(2)=-ext2(2);
    plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'g','LineWidth',2);
end
% %Swing foot
% -------------------------
% el tobillo tiene sus coordenadas conocidas en el marco 7
temp = T(1:3,4,7);  
x0f = temp(1) + x0;
y0f = temp(2) - y0;
z0f = temp(3);
% les coins ont des coordonnées fixes dans le repère 13
XAVG = T(:,:,7)*[-0.0460; 0.025; -0.025; 1]+[x0;-y0;0;0];%* Cambie el valor de -0.0674 a -0.04660
XAVD = T(:,:,7)*[-0.0460; 0.025; +0.025; 1]+[x0;-y0;0;0];%*
XARD = T(:,:,7)*[-0.0460; -0.015; 0.025; 1]+[x0;-y0;0;0];%*
XARG = T(:,:,7)*[-0.0460; -0.015; -0.025; 1]+[x0;-y0;0;0];%*
% Como deberia ser
% XAVG(3)=0;
% XAVD(3)=0;
% XARG(3)=0;
% XARD(3)=0;
% boucle : 1 = cheville, 2(avg)3 4 5 : 4 coins
% 1 1 1 1 2
% 2 3 2 4 3
% 3 4 5 5 4
% 3 4 5 5 5
xf=[x0f x0f x0f x0f XAVG(1);
    XAVG(1) XAVD(1) XAVG(1) XARG(1) XAVD(1);
    XAVD(1) XARD(1) XARG(1) XARD(1) XARD(1);
    XAVD(1) XARD(1) XARG(1) XARD(1) XARG(1)];
yf=[y0f y0f y0f y0f XAVG(2);
    XAVG(2) XAVD(2) XAVG(2) XARG(2) XAVD(2);
    XAVD(2) XARD(2) XARG(2) XARD(2) XARD(2);
    XAVD(2) XARD(2) XARG(2) XARD(2) XARG(2)];
zf=[z0f z0f z0f z0f XAVG(3);
    XAVG(3) XAVD(3) XAVG(3) XARG(3) XAVD(3);
    XAVD(3) XARD(3) XARG(3) XARD(3) XARD(3);
    XAVD(3) XARD(3) XARG(3) XARD(3) XARG(3)];
 h=patch(xf,yf,zf,'g');
 set(h,'edgecolor','g');

 
% %Torso
ext1 = T(1:3,4,4)+[x0;y0;0];
ext2 = T(1:3,4,9)+[x0;y0;0];
plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'LineWidth',2);
surf(x/200+ext2(1),y/200+ext2(2),z/200+ext2(3));
%
%Graficar los CoM de cada eslabón
%  if coms==1
%  CoM_j = robot.PI.CoM;
%  MS_j=zeros(3,8);
%  for j=1:8
%      MS_j(:,j) = T(1:3,:,j) * [CoM_j{j};1];
%      plot3(MS_j(1,j),MS_j(2,j),MS_j(3,j),'*');
%  end
%  end