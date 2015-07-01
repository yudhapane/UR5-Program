% PLOTLEARNING plot the current status of the RL algorithm for the UR5
% robot tracking
savedNumber = size(par.wTable);
figure;
for counter = 1: savedNumber(2)
	if strcmp(par.plotSelect,'1')
			% critic 
			positionVector1 = [0.05, 0.8, 0.43, 0.17];
			subplot('Position',positionVector1)
			plotOut = plotrbfUR5_4anim(par, 'critic', par.plotopt, counter); title(['\bf{CRITIC}  Iteration: ' int2str(trial_count)]); colorbar;
			xlabel('$error  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{error}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$V(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
			hold on; plot(par.weTable{counter}(3,:)*1000, -par.wdTable{counter}(3,:)*1000, 'r.');
			% plot(zeros(size(wdref)),wdref*1000,'b'); 
			% plot(linspace(par.xmin, par.xmax, size(wrTraj,2))*1000, zeros(1, size(wrTraj,2)));
			
			% actor
			positionVector2 = [0.05, 0.545, 0.43, 0.176];
			subplot('Position',positionVector2)
			plotOut = plotrbfUR5_4anim(par, 'actor', par.plotopt, counter); title('\bf{ACTOR}');  colorbar;
			hold on; plot(par.weTable{counter}(3,:)*1000, -par.wdTable{counter}(3,:)*1000, 'r.'); 
			% plot(zeros(size(wdref)),wdref*1000,'b');
			% plot(linspace(par.xmin, par.xmax, size(wrTraj,2))*1000, zeros(1, size(wrTraj,2)));
			xlabel('$error  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{error}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$\pi(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
			
			% position        
	%         positionVector3 = [0.05, 0.305, 0.43, 0.16];
			positionVector3 = [0.05, 0.063, 0.43, 0.36];
			subplot('Position',positionVector3)
			plot(par.time, 1000*par.wrTraj2{counter}(3,:), 'red'); hold on;
			plot(par.time,1000*par.wnomplotTable{counter}(3,:), 'green');   
			if (trial_count>0);            
			plot(par.time, 1000*par.wrTable{counter}(3,:), 'black');
			plot(par.time, 1000*par.wTable{counter}(3,:), 'blue');   
			end
			xlabel('par.time (seconds)'); ylabel('Z position (mm)'); xlim([0 par.time(end)]); title('reference (red), RL (blue), no-RL (green)');       
	%         legend('nominal reference', 'nominal trajectory', 'modified reference', 'trajectory');
			
			% velocity
	%         positionVector4 = [0.05, 0.063, 0.43, 0.16];
	%         subplot('Position',positionVector4)
	%         plot(par.time, 1000*wdTable(3,:));
	%         xlabel('par.time (seconds)'); ylabel('Z velocity (mm/s)'); xlim([0 par.t_end]); title('Z velocity trajectory');                      
			
			% temporal difference
			positionVector5 = [0.55, 0.72, 0.43, 0.24];
			subplot('Position',positionVector5)
			plot(par.time(1:end-1), par.deltaTable{counter}); title('\bf{Temporal difference}'); 
			hold on; plot(par.time(1:end-1), zeros(1, N-1));
			xlabel('par.time steps'); %ylim([-100 100]);
			
			% Return 
			positionVector6 = [0.55, 0.39, 0.41, 0.24];
			subplot('Position',positionVector6)
			tempRet = sum(par.Ret,2);
			plotyy(1:1:par.Ntrial, sum(par.Ret,2), 50:1:par.Ntrial, tempRet(50:end)); title('\bf{Return}');   %ylim([-10 5]);
			xlabel('trials');
					  

			% additive input and exploration
			if trial_count  % whenever learning trial has been committed
			positionVector7 = [0.55, 0.063, 0.43, 0.24];
			subplot('Position',positionVector7);
			plot(par.time(1:end), 1000*par.uadTable{counter}, par.time(1:end), 1000*par.du{counter}, 'r');
			xlabel('par.time (seconds)'); ylabel('additive modifier (mm)'); xlim([0 par.time(end)]); title('Additive modifier (b) & exploration (r)');       
			end
	elseif strcmp(par.plotSelect,'2')

	else
		error('Plot option is not recognized. Please assign the value either "1" or "2"');
    end
    pause(0.5);
    clf;
end