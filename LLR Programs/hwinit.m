%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot: RT blocks paramaters, I/O conversions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

adinoffs = [-443 -512 -562 -484 -522 -518    0];    % input offset 
adingain = [(pi*2/620/3)*[1 -1 -1 1 1 1] 1];    % input gain 

daoutoffs = 127*[1 1 1 1 1 1 0];                    % output offset
daoutgain = 127*[1 -1 -1 1 1 1 1];                  % output gain

ticklost = 100;
adinoffs = [-419 -513 -575 -511 -525 -519    0]; 
