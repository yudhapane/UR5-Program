function  DrawRobot(Z,H,t1,t2,t3,t4)

% DrawRobot: Redraws the polygons for a simplified 3D model of EdRo
%
% Usage:   DrawRobot(Z,H,t1,t2,t3,t4)
%
% Z  -  A collection of paths described by vectors that
%       represent each rigid body, created by function CreateRobot
% H  -  Collection of graphics handles for each rigid body created
%       by function CreateRobot
% t1 -  base angle
% t2 -  shoulder angle
% t3 -  elbow andle
% t4 -  wrist angle
% note: all angles in radians [-Pi, Pi]
% 
% See also CreateRobot
%
% History:  01/2009, G.A.D.Lopes@tudelft.nl
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% load polygon data
va=Z(:,1:3);
vb=Z(:,4:6);
vc=Z(:,7:9);
vd=Z(:,10:12);

% load graphics handlers
h1 =H(1);
h2 =H(2);
h3 =H(3);
h4 =H(4);

% forearm
vb=Ry(vb,-t2); % shoulder rotation
vb=Rz(vb,-t1); % base rotation

% arm
vc=Ry(vc,-t3); %elbow rotation
vc=vc+repmat([0 0 30],8,1); %translate
vc=Ry(vc,-t2); % shoulder rotation
vc=Rz(vc,-t1); % base rotation

% hand
vd=Ry(vd,-t4); %wrist rotation
vd=vd+repmat([25 0 0],8,1); %translate

vd=Ry(vd,-t3); %elbow rotation
vd=vd+repmat([0 0 30],8,1); %translate
vd=Ry(vd,-t2); % shoulder rotation
vd=Rz(vd,-t1); % base rotation

% update the graphics objects to their new locations
set(h1,'Vertices',va);
set(h2,'Vertices',vb);
set(h3,'Vertices',vc);
set(h4,'Vertices',vd);

end
