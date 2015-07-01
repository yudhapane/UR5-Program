%% make reference model

load measurements

x = [x(2:end,:)' ; x(2:end,:)'-x(1:end-1,:)'];      % deterministic state description
W = inv(diag(std( (x-diag(mean(x'))*ones(6,length(x)))' )));   % weighting for the state vector to normalize the sizes
%% concatenate [x(k) dx(k) | x(k+1) dx(k+1)]        
Ref = [x(:,1:end-1) ; x(:,2:end)];

%% detailed ref is: [x1(k) x2(k) x3(k) dx1(k) dx2(k) dx3(k) | x1(k+1) x2(k+1) x3(k+1) dx1(k+1) dx2(k+1) dx3(k+1)]        

save referencemodel Ref W

