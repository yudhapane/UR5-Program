DURATION        = 10;
SAMPLING_TIME   = 1/125;
NO_SAMPLES      = DURATION/SAMPLING_TIME;

time    = 0:SAMPLING_TIME:DURATION-SAMPLING_TIME;
% x0      = 0;
x0      = 0.3353;
refSig  = [x0*ones(1,400) (x0+0.005)*ones(1,400) x0*ones(1,450)];

s       = tf('s');
refTF   = 89/(s^2 + 10*s + 89);
refSS   = ss(refTF);
figure; 
lsim(refSS, refSig, time, [0 x0])