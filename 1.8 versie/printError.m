% clear; close all;
load('150114_104542_MPC.mat');
error_MPC = error_print(1+b:N-100);
load('150114_104423_P.mat');
error_PID = error_print(1+b:N-100);
figure;
plot(Time(1+b:N-100),error_P); hold on;
plot(Time(1+b:N-100),error_MPC,'r');
legend('P position error', 'MPC position error');
title('Printing error')
xlabel('Time [s]')
ylabel('Error in print [m]')
xlim([0 Tplot]); ylim([0 1e-3]);
