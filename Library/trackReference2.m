%TRACKREFERENCE2  execute trajectory with proportional controller
% 
% REFERENCETRACK2(ARM, STRAJ, SDOTTRAJ) executes a cartesian space
% trajectory tracking routine with 3 possibilities:
% 1. REFERENCETRACK2(ARM, QTRAJ) command robot ARM to execute position 
%    trajectory QTRAJ 
% 
% Yudha Prawira Pane (c)
% Created on Jan-22-2015
function [sTable, sdotTable, sedotTable] = trackReference2(arm, sTraj, sdotTraj)
    SAMPLING_TIME = 0.008;
    if nargin < 2
        error 'Not enough input argument(s)'
    end