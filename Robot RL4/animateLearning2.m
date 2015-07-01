% PLOTLEARNING plot the current status of the RL algorithm for the UR5
% robot tracking
close all; clear;
load('RLTest4d_2- par-26-Jun-15__12-56 PM val.mat');
N = length(par.wTable{1});
save_count = size(par.wTable);
figure;
disp('Starting learning animation ...');
for counter = 1: 1: save_count(2)    
    clf;
    % critic 
    positionVector1 = [0.05, 0.8, 0.43, 0.17];
    subplot('Position',positionVector1)
    plotOut = plotrbfUR5_4anim(par, 'critic', par.plotopt, counter); title(['\bf{CRITIC}  Iteration: ' int2str(counter)]); colorbar;
    xlabel('$error  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{error}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$V(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
    hold on; plot(par.weTable{counter}(3,:)*1000, -par.wdTable{counter}(3,:)*1000, 'r.');

    % actor
    positionVector2 = [0.05, 0.545, 0.43, 0.176];
    subplot('Position',positionVector2)
    plotOut = plotrbfUR5_4anim(par, 'actor', par.plotopt, counter); title('\bf{ACTOR}');  colorbar;
    hold on; plot(par.weTable{counter}(3,:)*1000, -par.wdTable{counter}(3,:)*1000, 'r.'); 
    xlabel('$error  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{error}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$\pi(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 

    % position        
%         positionVector3 = [0.05, 0.305, 0.43, 0.16];
    positionVector3 = [0.05, 0.063, 0.43, 0.36];
    subplot('Position',positionVector3)
    plot(par.time, 1000*par.wrTraj2{counter}(3,:), 'red'); hold on;
    plot(par.time,1000*par.wnomplotTable{counter}(3,:), 'green');  
    plot(par.time, 1000*par.wTable{counter}(3,:), 'blue');   grid on;
    xlabel('time (seconds)'); ylabel('Z position (mm)'); xlim([0 par.time(end)]); title('reference (red), RL (blue), no-RL (green)');       

    % temporal difference
    positionVector5 = [0.55, 0.72, 0.43, 0.24];
    subplot('Position',positionVector5)
    plot(par.time(1:end-1), par.deltaTable{counter}); title('\bf{Temporal difference}'); grid on;
% 			hold on; plot(par.time(1:end-1), zeros(1, N-1));
    xlabel('time steps'); %ylim([-100 100]);

    % Return 
    positionVector6 = [0.55, 0.39, 0.41, 0.24];
    subplot('Position',positionVector6)
    sumRet = sum(par.Ret,2);
    plot(sumRet(1:counter)); title('\bf{Return}'); grid on;
    xlim([1 par.Ntrial]);
    xlabel('trials');


    % reference
    positionVector7 = [0.55, 0.063, 0.43, 0.24];
    subplot('Position',positionVector7);       
    plot(par.time, 1000*par.wrTraj2{counter}(3,:), 'red'); hold on; grid on;
    plot(par.time, 1000*par.wrTable{counter}(3,:), 'blue');
    xlabel('time (seconds)'); ylabel('Z position (mm)'); xlim([0 par.time(end)]); title('Old (red) and Modified (blue) Reference');       
    if counter == 1
        pause;
    else
        pause(0.5);
    end 
end
					
figure;
subplot(211);
plot(par.time, par.wrTraj2{end}(3,:)); grid on;
hold on;
plot(par.time, par.wTable{end}(3,:));
plot(par.time, par.wnomplotTable{end}(3,:)); 
legend('reference', 'trajectory (RL)', 'trajectory (nominal)');
title('Tracking comparison');
grid on; xlabel('time (seconds)'); ylabel('z-axis trajectory');
subplot(212);
plot(par.time, par.wrTraj2{end}(3,:)); grid on;
hold on;
plot(par.time, par.wrTable{end}(3,:));
legend('old reference', 'modified reference');
title('Modified reference');
xlabel('time (seconds)'); ylabel('z-axis reference');

format long;
err2 = rms(par.wrTraj2{end}(3,:)-par.wnomplotTable{end}(3,:));
err1 = rms(par.wrTraj2{end}(3,:)-par.wTable{end}(3,:));
RMS_improvement = (err2-err1)/err2;
err2 = par.wTable{end}(3,end) - par.wrTraj2{end}(3,end);
err2 = par.wnomplotTable{end}(3,end) - par.wrTraj2{end}(3,end);
SS_error_improvement = (err2-err1)/err2;

fprintf('The improvement in total RMS error is: %s percent\n', num2str(RMS_improvement*100));
fprintf('The improvement in steady state error is: %s percent \n', num2str(abs(SS_error_improvement)*100));


%% Display Different Trajectory
disp('Policy performance for different reference ...');
% note that the learning is not performed again, i.e. old policy is used
% for new trajectory
load('RLTest4d_2- par-26-Jun-15__15-07 PM val.mat');
save_count = size(par.wTable);
figure;
% critic 
positionVector1 = [0.05, 0.8, 0.43, 0.17];
subplot('Position',positionVector1)
plotOut = plotrbfUR5_4anim(par, 'critic', par.plotopt, save_count); title('\bf{CRITIC}'); colorbar;
xlabel('$error  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{error}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$V(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
hold on; plot(par.weTable{end}(3,:)*1000, -par.wdTable{end}(3,:)*1000, 'r.');

% actor
positionVector2 = [0.05, 0.545, 0.43, 0.176];
subplot('Position',positionVector2)
plotOut = plotrbfUR5_4anim(par, 'actor', par.plotopt, save_count); title('\bf{ACTOR}');  colorbar;
hold on; plot(par.weTable{end}(3,:)*1000, -par.wdTable{end}(3,:)*1000, 'r.'); 
xlabel('$error  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{error}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$\pi(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 

% position        
%         positionVector3 = [0.05, 0.305, 0.43, 0.16];
positionVector3 = [0.05, 0.063, 0.43, 0.36];
subplot('Position',positionVector3)
plot(par.time, 1000*par.wrTraj2{end}(3,:), 'red'); hold on;
plot(par.time,1000*par.wnomplotTable{end}(3,:), 'green');  
plot(par.time, 1000*par.wTable{end}(3,:), 'blue');   grid on;
xlabel('time (seconds)'); ylabel('Z position (mm)'); xlim([0 par.time(end)]); title('reference (red), RL (blue), no-RL (green)');       

% temporal difference
positionVector5 = [0.55, 0.72, 0.43, 0.24];
subplot('Position',positionVector5)
plot(par.time(1:end-1), par.deltaTable{end}); title('\bf{Temporal difference}'); grid on;
% 			hold on; plot(par.time(1:end-1), zeros(1, N-1));
xlabel('time steps'); %ylim([-100 100]);

% Return 
positionVector6 = [0.55, 0.39, 0.41, 0.24];
subplot('Position',positionVector6)
sumRet = sum(par.Ret,2);
plot(sumRet(1:end)); title('\bf{Return}'); grid on;
xlim([1 par.Ntrial]);
xlabel('trials');


% reference
positionVector7 = [0.55, 0.063, 0.43, 0.24];
subplot('Position',positionVector7);       
plot(par.time, 1000*par.wrTraj2{end}(3,:), 'red'); hold on; grid on;
plot(par.time, 1000*par.wrTable{end}(3,:), 'blue');
xlabel('time (seconds)'); ylabel('Z position (mm)'); xlim([0 par.time(end)]); title('Old (red) and Modified (blue) Reference');       
pause(0.5);

					
figure;
subplot(211);
plot(par.time, par.wrTraj2{end}(3,:)); grid on;
hold on;
plot(par.time, par.wTable{end}(3,:));
plot(par.time, par.wnomplotTable{end}(3,:)); 
legend('reference', 'trajectory (RL)', 'trajectory (nominal)');
title('Tracking comparison');
grid on; xlabel('time (seconds)'); ylabel('z-axis trajectory');
subplot(212);
plot(par.time, par.wrTraj2{end}(3,:)); grid on;
hold on;
plot(par.time, par.wrTable{end}(3,:));
legend('old reference', 'modified reference');
title('Modified reference');
xlabel('time (seconds)'); ylabel('z-axis reference');