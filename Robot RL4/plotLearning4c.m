% PLOTLEARNING plot the current status of the RL algorithm for the UR5
% robot tracking

if strcmp(par.plotSelect,'1')
        % critic 
        positionVector1 = [0.05, 0.8, 0.43, 0.17];
        subplot('Position',positionVector1)
        plotOut = plotrbfUR5_4c(par, 'critic', par.plotopt); title(['\bf{CRITIC}  Iteration: ' int2str(trial_count)]); colorbar;
        xlabel('$error  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$reference  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$V(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
        hold on; plot(weTable(3,:), wrTraj2(3,:), 'r.');
        plot(zeros(size(wdref)),wdref,'b'); 
        plot(linspace(par.xmin, par.xmax, size(wrTraj,2)), zeros(1, size(wrTraj,2)));
        
        % actor
        positionVector2 = [0.05, 0.545, 0.43, 0.176];
        subplot('Position',positionVector2)
        plotOut = plotrbfUR5_4c(par, 'actor', par.plotopt); title('\bf{ACTOR}');  colorbar;
        hold on; plot(weTable(3,:), wrTraj2(3,:), 'r.'); 
        plot(zeros(size(wdref)),wdref,'b');
        plot(linspace(par.xmin, par.xmax, size(wrTraj,2)), zeros(1, size(wrTraj,2)));
        xlabel('$error  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$reference  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$\pi(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
        
        % position        
%         positionVector3 = [0.05, 0.305, 0.43, 0.16];
        positionVector3 = [0.05, 0.063, 0.43, 0.36];
        subplot('Position',positionVector3)
        deltaZ = wrTraj2(3,:)-wrTraj(3,:);
        wnomplotTable(3,:) = wnomTable(3,:)+deltaZ;
        plot(time, 1000*wrTraj2(3,:), 'red'); hold on;
        plot(time,1000*wnomplotTable(3,:), 'green');   
        if (trial_count>0);            
        plot(time, 1000*wrTable(3,:), 'black');
        plot(time, 1000*wTable(3,:), 'blue');   
        end
        xlabel('time (seconds)'); ylabel('Z position (mm)'); xlim([0 time(end)]); title('reference (red), RL (blue), no-RL (green)');       
%         legend('nominal reference', 'nominal trajectory', 'modified reference', 'trajectory');
        
        % velocity
%         positionVector4 = [0.05, 0.063, 0.43, 0.16];
%         subplot('Position',positionVector4)
%         plot(time, 1000*wdTable(3,:));
%         xlabel('time (seconds)'); ylabel('Z velocity (mm/s)'); xlim([0 par.t_end]); title('Z velocity trajectory');                      
        
        % temporal difference
        positionVector5 = [0.55, 0.72, 0.43, 0.24];
        subplot('Position',positionVector5)
        plot(time(1:end-1), delta); title('\bf{Temporal difference}'); 
        hold on; plot(time(1:end-1), zeros(1, N-1));
        xlabel('time steps'); %ylim([-100 100]);
        
        % Return 
        positionVector6 = [0.55, 0.39, 0.41, 0.24];
        subplot('Position',positionVector6)
        tempRet = sum(Ret,2);
        plotyy(1:1:par.Ntrial, sum(Ret,2), 11:1:par.Ntrial, tempRet(11:end)); title('\bf{Return}');   %ylim([-10 5]);
        xlabel('trials');
                  

        % additive input and exploration
        if trial_count  % whenever learning trial has been committed
        positionVector7 = [0.55, 0.063, 0.43, 0.24];
        subplot('Position',positionVector7);
        plot(time(1:end-1), 1000*dref, time(1:end-1), 1000*du(1:end-1), 'r');
        xlabel('time (seconds)'); ylabel('additive modifier (mm)'); xlim([0 time(end)]); title('Additive modifier (b) & exploration (r)');       
        end
elseif strcmp(par.plotSelect,'2')

else
    error('Plot option is not recognized. Please assign the value either "1" or "2"');
end