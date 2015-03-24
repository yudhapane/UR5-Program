%WORK2JOINTSPACE  convert workspace to joint space trajectory for UR5 robot
% 
% QTRAJ = WORK2JOINTSPACE(PTRAJ) convert workspace position trajectory (PTRAJ)
% to joint space position trajectory (QTRAJ)
% 
% [QTRAJ, QDOTTRAJ] = WORK2JOINTSPACE(PTRAJ, PDOTTRAJ) convert workspace 
% trajectory for both position (PTRAJ) and velocity (PDOTTRAJ) to joint space
% trajectory position (QTRAJ) and velocity (QDOTTRAJ)
% 
% Reference:  This function used robotics toolbox (rvc toolbox) from 
% Peter I. Corke (http://www.petercorke.com/Robotics_Toolbox.html_)
% 
% Yudha Prawira Pane (c)
% Created on Jan-14-2015

function [qTraj, qdotTraj] = work2JointSpace(pTraj, pdotTraj)
    if nargin < 1
        error('Not enough input argument(s)');
    end
    
    
        