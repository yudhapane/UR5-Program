%% Dummy file, don't run it will not work

ac = (uAlg+delUAll);    % control action given by the actor (algorithm) + exploration
extNegRew = 0;          % assume the system is within safe operating region
for kk=1:sys.Nu
    if((x(k)>sys.qmSafe.Low) && (ac>0))
        % system state for at even one link is above the safe bound
        % the control action is also positive, i.e. the actual control
        % pushes the system towards edge recalculate control 
        % give extra negative reward
        delUAll = -uAlg;
        ac = 0;
        extNegRew=extNegRew+1;
    end
    if((x(k)<-sys.qmSafe.Low) && (ac<0))
        % similarly for lower bound
        delUAll = -uAlg;
        ac = 0;
        extNegRew=extNegRew+1;
    end
end
% apply control input ac
% read next states
% calculate reward
reward = ((x(k)-par.xdes')*-par.Q*(x(k)-par.xdes')'-ac*par.R*ac'-extNegRew*10); 
% it is quadratic reward + and extra term whenever there is a overshoot
% the overshoot is set based on

%update critic and actor parameters
