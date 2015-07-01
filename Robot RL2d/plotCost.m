% loadParamsUR5_2;
N = 50;
xdata = linspace(params.zllim,params.zulim, N);
xdotdata = linspace(params.zdotllim, params.zdotulim, N);
costData = zeros(N,N);
zref = 0.335349586756066;
% clf;
% subplot(211);
for i = 1: N
    for j = 1: N
        costData(i,j) = costUR5_2(xdata(j), xdotdata(i), 0, 0.335349586756066, params);
    end
end
figure(2); surf(xdata, xdotdata, costData);
% shading interp;

% subplot(212); 
% x = -pi:0.01:pi;
% % plot(x,tan(10*x)); 
% plot(xdata, -2e3+0.8e2*tan(5*zref + 62*(xdata-zref))); hold on;
% plot(zref*ones(size(xdata)), linspace(-1000,10,length(xdata)));
% plot(0.332*ones(size(xdata)), linspace(-1000,1e3,length(xdata)));
% 
% plot(xdata, zeros(size(xdata)));
% % plot(x, tan(pi/2+2*x));        
% xlim([xdata(1) xdata(end)]);
% ylim([-1e3 1e3]);