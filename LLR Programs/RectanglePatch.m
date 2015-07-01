function [vertices faces] = RectanglePatch(xl,yl,zl,xo,yo,zo)
% created by G. Lopes January 2009 
%
% This function generates 3D boxes with size xl yl zl and centered at xo yo zo
%

vertices = [-1 -1 -1;
             1 -1 -1;
             1  1 -1;
            -1  1 -1;
            -1 -1  1;
             1 -1  1;
             1  1  1;
            -1  1  1]*[xl/2 0 0; 0 yl/2 0 ; 0 0 zl/2]+repmat([xo yo zo],8,1);
         
faces=[1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4 ; 5 6 7 8];
        
end