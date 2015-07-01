function [wrTraj, wrdTraj, time, N] = genTraj(opt, par, arm)
    EXPERIMENT_TIME = 10;
    SAMPLING_TIME   = par.ts;
    if opt == 1
        N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];
        arm.update();
        w0              = arm.getToolPositions();   % initial cartesian pose

        wrTraj     	= zeros(6,N);   % reference trajectory, will be populated in the iteration
        offset    	= [-0.25 0 0 0 0 0]';
        dOffset    	= offset/N;

        for i = 1:N
            wrTraj(:,i)	= w0 + (i-1)*dOffset; % populate reference trajectory
        end
         wrdTraj            = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
         wrdTraj(:,end+1)   = zeros(3,1);
    elseif opt == 2
        n1              = 3/SAMPLING_TIME;
        n2              = 7/SAMPLING_TIME;
        n3              = n1;
        N               = n1+n2+n3;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();

        wrTraj(:,1:n1)  = repmat(w0,[1,n1]);
        wrTraj(1,1:n1)  = linspace(w0(1), w0(1)-0.10, n1);
        wrTraj(3,1:n1)  = linspace(w0(3), w0(3)+0.01, n1);
        wrTraj(:,n1+1:n1+n2)    = repmat(wrTraj(:,n1),[1,n2]);
        wrTraj(1,n1+1:n1+n2)    = linspace(wrTraj(1,n1), wrTraj(1,n1)-0.05, n2);
        wrTraj(:,n1+n2+1:n1+n2+n3)    = repmat(wrTraj(:,n1+n2),[1,n3]);
        wrTraj(1,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(1,n1+n2), wrTraj(1,n1+n2)-0.10, n3);
        wrTraj(3,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(3,n1+n2), wrTraj(3,n1+n2)-0.01, n3);
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);
	elseif opt == 3
        N               = 3/SAMPLING_TIME;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();

        wrTraj(:,1:N)  = repmat(w0,[1,N]);
        wrTraj(1,1:N)  = linspace(w0(1), w0(1)-0.05, N);
        wrTraj(3,1:N)  = linspace(w0(3), w0(3)+0.01, N);
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);	
	elseif opt == 4
        N               = 3/SAMPLING_TIME;
        EXPERIMENT_TIME = N*par.ts;
        time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
        time(end)       = [];

        arm.update();
        w0  	= arm.getToolPositions();

        wrTraj(:,1:N)  = repmat(w0,[1,N]);
        wrTraj(1,1:N)  = linspace(w0(1), w0(1)-0.10, N);
        wrTraj(3,1:N)  = linspace(w0(3), w0(3)-0.005, N);
        
        wrdTraj             = (wrTraj(1:3,2:end)-wrTraj(1:3,1:end-1))/SAMPLING_TIME;
        wrdTraj(:,end+1)    = zeros(3,1);
    end