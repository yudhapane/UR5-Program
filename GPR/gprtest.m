disp('See http://www.gaussianprocess.org/gpml/code/matlab/doc/ for details.')
disp('Hit any key to continue...'); pause(0.5);

disp(' '); disp('clear all, close all')
clear all, close all;
write_fig = 0;
disp(' ')

%% specify the GP's functions and hyperparameters
disp('meanfunc = {@meanSum, {@meanLinear, @meanConst}}; hyp.mean = [0.5; 1];')
meanfunc = {@meanSum, {@meanLinear, @meanConst}}; hyp.mean = [0.5; 0.3];
disp('covfunc = {@covMaterniso, 3}; ell = 1/4; sf = 1; hyp.cov = log([ell; sf]);')
covfunc = {@covMaterniso, 5}; ell = 5e-4; sf = 1e-3; hyp.cov = log([ell; sf]);
disp('likfunc = @likGauss; sn = 0.1; hyp.lik = log(sn);')
likfunc = @likGauss; sn = 4e-4; hyp.lik = log(sn);
disp(' ')

load('D:\Dropbox\TU Delft - MSc System & Control\Graduation Project (Thesis)\UR5 Robot\UR5 Programs\Robot RL2c\parameters\moderate result\par-12-May-15__13-15 PM.mat');
x1 = par.uadTable{200}; x1 = x1';
x2 = par.uadTable{10} ; x2 = x2';
x  = [x1; x2];
y1 = par.wTable{200}(3,2:end); y1 = y1';
y2 = par.wTable{10}(3,2:end) ; y2 = y2';
y  = [y1; y2];

x  = par.uadTable{200}; x = x';
y = par.wTable{200}(3,2:end); y = y';

n = length(x);
disp(['n = ;' num2str(n)]);
disp('K = feval(covfunc{:}, hyp.cov, x);')
K = feval(covfunc{:}, hyp.cov, x);
disp('mu = feval(meanfunc{:}, hyp.mean, x);')
mu = feval(meanfunc{:}, hyp.mean, x);

%% plot training set
figure(1)
set(gca, 'FontSize', 24)
disp(' '); disp('plot(x, y, ''+'')')
plot(x, y, '+', 'MarkerSize', 12)
grid on
xlabel('input, x')
ylabel('output, y')
if write_fig, print -depsc f1.eps; end
disp(' '); disp('Hit any key to continue...'); pause(0.5);

%% train GPR
disp(' ')
disp('nlml = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y)')
nlml = gp(hyp, @infExact  , meanfunc, covfunc, likfunc, x, y)
disp(' ')

%% validation #1
disp('z = linspace(-1.9, 1.9, 101)'';')
z = linspace(min(x), max(x), 101)';
disp('[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);')
[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);

%% Plot validation #1
figure(2)
set(gca, 'FontSize', 24)
disp(' ')
disp('f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];') 
f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
disp('fill([z; flipdim(z,1)], f, [7 7 7]/8);')
fill([z; flipdim(z,1)], f, [7 7 7]/8);

disp('hold on; plot(z, m); plot(x, y, ''+'')')
hold on; plot(z, m, 'LineWidth', 2); plot(x, y, '+', 'MarkerSize', 12)
grid on
xlabel('input, x')
ylabel('output, y')
if write_fig, print -depsc f2.eps; end
disp(' '); disp('Hit any key to continue...'); pause(0.5);

%% validation #2
disp('z = linspace(-1.9, 1.9, 101)'';')
z = par.uadTable{100}; z = z'; %z = sort(z); 
disp('[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);')

tic
[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);
toc
%% Plot validation #2
figure(2)
set(gca, 'FontSize', 24)
disp(' ')
disp('f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];') 
f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
disp('fill([z; flipdim(z,1)], f, [7 7 7]/8);')
fill([z; flipdim(z,1)], f, [7 7 7]/8);

disp('hold on; plot(z, m); plot(x, y, ''+'')')
hold on; plot(z, m, 'LineWidth', 2); plot(x, y, '+', 'MarkerSize', 12)
grid on
xlabel('input, x')
ylabel('output, y')
if write_fig, print -depsc f2.eps; end
disp(' '); disp('Hit any key to continue...'); pause

%% GPR improvement
disp(' ')
disp('covfunc = @covSEiso; hyp2.cov = [0; 0]; hyp2.lik = log(0.1);')
covfunc = @covSEiso; hyp2.cov = [0; 0]; hyp2.lik = log(0.1);
disp('hyp2 = minimize(hyp2, @gp, -100, @infExact, [], covfunc, likfunc, x, y)')
hyp2 = minimize(hyp2, @gp, -100, @infExact, [], covfunc, likfunc, x, y);
disp(' ')

disp('exp(hyp2.lik)')
exp(hyp2.lik)
disp('nlml2 = gp(hyp2, @infExact, [], covfunc, likfunc, x, y)')
nlml2 = gp(hyp2, @infExact, [], covfunc, likfunc, x, y)
disp('[m s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, z);')
[m s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, z);

disp(' ')
figure(3)
set(gca, 'FontSize', 24)
f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
disp('fill([z; flipdim(z,1)], f, [7 7 7]/8)');
fill([z; flipdim(z,1)], f, [7 7 7]/8)
disp('hold on; plot(z, m); plot(x, y, ''+'')');
hold on; plot(z, m, 'LineWidth', 2); plot(x, y, '+', 'MarkerSize', 12)
grid on
xlabel('input, x')
ylabel('output, y')
% axis([-1.9 1.9 -0.9 3.9])
if write_fig, print -depsc f3.eps; end
disp(' '); disp('Hit any key to continue...'); pause

disp(' ')
disp('hyp.cov = [0; 0]; hyp.mean = [0; 0]; hyp.lik = log(0.1);')
hyp.cov = [0; 0]; hyp.mean = [0; 0]; hyp.lik = log(0.1);
disp('hyp = minimize(hyp, @gp, -100, @infExact, meanfunc, covfunc, likfunc, x, y);')
hyp = minimize(hyp, @gp, -100, @infExact, meanfunc, covfunc, likfunc, x, y);
disp('[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);')
[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);

figure(4)
set(gca, 'FontSize', 24)
disp(' ')
disp('f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];')
f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
disp('fill([z; flipdim(z,1)], f, [7 7 7]/8)')
fill([z; flipdim(z,1)], f, [7 7 7]/8)
disp('hold on; plot(z, m); plot(x, y, ''+'');')
hold on; plot(z, m, 'LineWidth', 2); plot(x, y, '+', 'MarkerSize', 12)
grid on
xlabel('input, x')
ylabel('output, y')
% axis([-1.9 1.9 -0.9 3.9])
if write_fig, print -depsc f4.eps; end
disp(' '); disp('Hit any key to continue...'); pause

disp('large scale regression using the FITC approximation')
disp('nu = fix(n/2); u = linspace(-1.3,1.3,nu)'';')
nu = fix(n/2); u = linspace(-1.3,1.3,nu)';
disp('covfuncF = {@covFITC, {covfunc}, u};')
covfuncF = {@covFITC, {covfunc}, u};
disp('[mF s2F] = gp(hyp, @infFITC, meanfunc, covfuncF, likfunc, x, y, z);')
[mF s2F] = gp(hyp, @infFITC, meanfunc, covfuncF, likfunc, x, y, z);

figure(5)
set(gca, 'FontSize', 24)
disp(' ')
disp('f = [mF+2*sqrt(s2F); flipdim(mF-2*sqrt(s2F),1)];')
f = [mF+2*sqrt(s2F); flipdim(mF-2*sqrt(s2F),1)];
disp('fill([z; flipdim(z,1)], f, [7 7 7]/8)')
fill([z; flipdim(z,1)], f, [7 7 7]/8)
disp('hold on; plot(z, mF); plot(x, y, ''+'');')
hold on; plot(z, mF, 'LineWidth', 2); plot(x, y, '+', 'MarkerSize', 12)
disp('plot(u,1,''o'')')
plot(u,1,'ko', 'MarkerSize', 12)
grid on
xlabel('input, x')
ylabel('output, y')
% axis([-1.9 1.9 -0.9 3.9])
if write_fig, print -depsc f5.eps; end
disp(' ')