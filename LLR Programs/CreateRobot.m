function [Z H] = CreateRobot()

% CreateRobot: Creates polygons for a simplified 3D model
%              of the EdRo robot
%
% Usage:   [Z H] = CreateRobot()
%
% Z  -  A collection of paths described by vectors that
%       represent each rigid body
% H  -  Collection of graphics handles for each rigid body
% 
% See also DrawRobot
%
% History:  01/2009, G.A.D.Lopes@tudelft.nl
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% create initial patches
[va fa]=RectanglePatch(45,30,2,-7.5,0,0);
[vb fb]=RectanglePatch(5,18,30,0,0,15);
[vc fc]=RectanglePatch(42,10,5,7,0,0);
[vd fd]=RectanglePatch(10,8,3,5,0,0);

Z=[va vb vc vd];

t1=pi/180*0;
t2=pi/180*0;
t3=pi/180*0;
t4=pi/180*0;

% forearm
vb=Ry(vb,t2); % shoulder rotation
vb=Rz(vb,t1); % base rotation

% arm
vc=Ry(vc,t3); %elbow rotation
vc=vc+repmat([0 0 30],8,1); %translate
vc=Ry(vc,t2); % shoulder rotation
vc=Rz(vc,t1); % base rotation

% hand
vd=Ry(vd,t4); %wrist rotation
vd=vd+repmat([25 0 0],8,1); %translate

vd=Ry(vd,t3); %elbow rotation
vd=vd+repmat([0 0 30],8,1); %translate
vd=Ry(vd,t2); % shoulder rotation
vd=Rz(vd,t1); % base rotation

%draw patches
h1=patch('Vertices',va,'Faces',fa,'FaceColor',[.5,.5,.5],'EdgeColor','none');
h2=patch('Vertices',vb,'Faces',fb,'FaceColor',[.5,.5,.5],'EdgeColor','none');
h3=patch('Vertices',vc,'Faces',fc,'FaceColor',[0,0,1],'EdgeColor','none');
h4=patch('Vertices',vd,'Faces',fd,'FaceColor',[0,0,1],'EdgeColor','none');


H =[ h1 h2 h3 h4];

% render in 3D
lightangle(45,30); 
set(gcf,'Renderer','OpenGL');
view(3);
camproj perspective;
daspect([1 1 1]); % fix aspect ratio



end
